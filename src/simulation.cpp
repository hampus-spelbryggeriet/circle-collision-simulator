#include "simulation.h"

#include <immintrin.h>
#include <sys/types.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <mutex>

#include "raylib.h"
#include "raymath.h"
#include "simd.h"

#define VECTORIZATION 256
#define VECTOR_BYTES (VECTORIZATION / 8)

struct Line {
    Vector2 point;
    Vector2 direction;
};

inline float frand() {
    return float(std::rand()) / float(RAND_MAX);
}

inline void swap(int* __restrict__ x, int* __restrict__ y) {
    int temp = *x;
    *x = *y;
    *y = temp;
}

template<typename T>
inline T* new_vector_array(size_t count) {
    count = (count + VECTOR_BYTES - 1) / VECTOR_BYTES * VECTOR_BYTES;
    return new (std::align_val_t(VECTOR_BYTES)) T[count];
}

SimulationState::SimulationState(int num_elements, float spawn_rate) {
    this->num_elements = num_elements;
    this->spawn_rate = spawn_rate;
    this->spawn_timer = 1.0f / spawn_rate;
    positions_x = new_vector_array<float>(num_elements);
    positions_y = new_vector_array<float>(num_elements);
    velocities_x = new_vector_array<float>(num_elements);
    velocities_y = new_vector_array<float>(num_elements);
    radii = new_vector_array<float>(num_elements);
    masses = new_vector_array<float>(num_elements);
    draw_positions = new Vector2[this->num_elements];
    colors = new Color[this->num_elements];
}

SimulationState::~SimulationState() {
    delete[] draw_positions;
    delete[] colors;
    delete[] masses;
    delete[] radii;
    delete[] velocities_y;
    delete[] velocities_x;
    delete[] positions_y;
    delete[] positions_x;
}

void SimulationState::init_random(const SimulationParameters& params) {
    for (int i = 0; i < num_elements; i += 1) {
        radii[i] = params.min_radius + (params.max_radius - params.min_radius) * frand();
        masses[i] = PI * radii[i] * radii[i];
        positions_x[i] = radii[i] + (params.width - 2.0f * radii[i]) * frand();
        positions_y[i] = radii[i] + (params.height - 2.0f * radii[i]) * frand();
        velocities_x[i] = 0.0f;
        velocities_y[i] = 0.0f;
        colors[i] = {uint8_t(128 + 127 * frand()), uint8_t(128 + 127 * frand()), uint8_t(128 + 127 * frand()), 255};
    }
}

void SimulationState::update(float delta_time, const Vector2& gravity, const Vector2& bounds) {
    float remaining_time = delta_time;

    while (visible_elements < num_elements && spawn_timer < 0.0f) {
        insert_markers(visible_elements, remaining_time);
        visible_elements = std::min(visible_elements + 1, num_elements);
        spawn_timer += 1.0f / spawn_rate;
    }

    update_velocities(remaining_time, gravity);
    update_markers(remaining_time);

    while (true) {
        active_elements.clear();
        sweep_elements.clear();
        target_elements.clear();

        for (int marker : markers) {
            const size_t index = calculate_marker_index(marker);
            if (marker < 0) {
                for (size_t active_element : active_elements) {
                    const Vector2 position = {positions_x[index], positions_y[index]};
                    const Vector2 velocity = {velocities_x[index], velocities_y[index]};
                    const Vector2 half_movement = velocity * 0.5 * remaining_time;
                    const float inflation = Vector2Length(half_movement);
                    const Vector2 active_position = {positions_x[active_element], positions_y[active_element]};
                    const Vector2 active_velocity = {velocities_x[active_element], velocities_y[active_element]};
                    const Vector2 active_half_movement = active_velocity * 0.5 * remaining_time;
                    const float active_inflation = Vector2Length(active_half_movement);
                    const float bounding_distance_squared =
                        Vector2LengthSqr(position + half_movement - (active_position + active_half_movement));
                    const float contact_distance = radii[index] + radii[active_element] + inflation + active_inflation;

                    if (bounding_distance_squared <= contact_distance * contact_distance) {
                        sweep_elements.push_back(index);
                        target_elements.push_back(active_element);
                    }
                }

                constexpr int stride = VECTOR_BYTES / sizeof(float);
                sweep_elements.resize((sweep_elements.size() + stride - 1) / stride * stride, 0);
                target_elements.resize((target_elements.size() + stride - 1) / stride * stride, 0);
                active_elements.push_back(index);
            } else {
                active_elements.remove(index);
            }
        }

        collision_event.time_hit = remaining_time;

        sweep_circle_to_circle();
        sweep_circle_to_edge(EdgeTarget::Left, bounds, remaining_time);
        sweep_circle_to_edge(EdgeTarget::Right, bounds, remaining_time);
        sweep_circle_to_edge(EdgeTarget::Top, bounds, remaining_time);
        sweep_circle_to_edge(EdgeTarget::Bottom, bounds, remaining_time);

        const bool did_collide = collision_event.time_hit < remaining_time;
        if (did_collide) {
            update_positions(collision_event.time_hit - TIME_BIAS, bounds);

            switch ((EdgeTarget)collision_event.target) {
                case EdgeTarget::Left:
                case EdgeTarget::Right:
                    velocities_x[collision_event.collider] *= -1.0f;
                    break;
                case EdgeTarget::Top:
                case EdgeTarget::Bottom:
                    velocities_y[collision_event.collider] *= -1.0f;
                    break;
                default:
                    const Vector2 relative_velocity = {
                        velocities_x[collision_event.target] - velocities_x[collision_event.collider],
                        velocities_y[collision_event.target] - velocities_y[collision_event.collider],

                    };
                    const Vector2 offset_hit = {
                        positions_x[collision_event.target] - positions_x[collision_event.collider],
                        positions_y[collision_event.target] - positions_y[collision_event.collider],
                    };
                    const float distance_hit = Vector2Length(offset_hit);
                    const Vector2 reflect_normal = offset_hit / (distance_hit * distance_hit);
                    const float total_mass = masses[collision_event.collider] + masses[collision_event.target];
                    const float k = 2.0f * Vector2DotProduct(relative_velocity, offset_hit) / total_mass;

                    velocities_x[collision_event.collider] += reflect_normal.x * k * masses[collision_event.target];
                    velocities_y[collision_event.collider] += reflect_normal.y * k * masses[collision_event.target];
                    velocities_x[collision_event.target] -= reflect_normal.x * k * masses[collision_event.collider];
                    velocities_y[collision_event.target] -= reflect_normal.y * k * masses[collision_event.collider];
                    break;
            }

            remaining_time -= collision_event.time_hit;
            update_markers(remaining_time);
        } else {
            update_positions(remaining_time, bounds);
            break;
        }
    }

    copy_positions();

    spawn_timer -= delta_time;
}

void SimulationState::draw(const Vector2& bounds) {
    std::lock_guard<std::mutex> guard(copy_mutex);

    for (int i = 0; i < visible_elements; i += 1) {
        DrawCircle(draw_positions[i].x, bounds.y - draw_positions[i].y, radii[i], colors[i]);
    }
}

void SimulationState::update_markers(float delta_time) {
    sort_markers(0, markers.size() - 1, delta_time);
}

void SimulationState::insert_markers(size_t i, float delta_time) {
    insert_marker(-(i + 1), delta_time);
    insert_marker(i + 1, delta_time);
}

void SimulationState::insert_marker(int marker, float delta_time) {
    const float marker_key = calculate_marker_key(marker, delta_time);

    for (std::vector<int>::const_iterator it = markers.cbegin(); it != markers.cend(); ++it) {
        if (marker_key >= calculate_marker_key(*it, delta_time)) {
            continue;
        }
        markers.insert(it, marker);
        return;
    }

    markers.push_back(marker);
}

void SimulationState::sort_markers(int low, int high, float delta_time) {
    if (low < high) {
        swap(&markers[(low + high) / 2], &markers[high]);

        const float pivot = calculate_marker_key(markers[high], delta_time);

        int p = low;
        for (int i = low; i < high; i += 1) {
            if (calculate_marker_key(markers[i], delta_time) <= pivot) {
                swap(&markers[p], &markers[i]);
                p += 1;
            }
        }

        swap(&markers[p], &markers[high]);

        sort_markers(low, p - 1, delta_time);
        sort_markers(p + 1, high, delta_time);
    }
}

size_t SimulationState::calculate_marker_index(int marker) {
    return std::abs(marker) - 1;
}

float SimulationState::calculate_marker_key(int marker, float delta_time) {
    const size_t index = calculate_marker_index(marker);
    const int marker_direction = (0 < marker) - (marker < 0);
    const int velocity_direction = (0 < velocities_y[index]) - (velocities_y[index] < 0);
    float key = positions_y[index];
    key += (marker_direction == velocity_direction) * velocities_y[index] * delta_time;
    key += marker_direction * (radii[index]);
    return key;
}

void SimulationState::update_velocities(float delta_time, const Vector2& gravity) {
    // Vectorize constants
    const __m256 increment_x = vsplat(gravity.x * delta_time);
    const __m256 increment_y = vsplat(gravity.y * delta_time);

    constexpr int stride = VECTOR_BYTES / sizeof(float);
    for (int i = 0; i < visible_elements; i += stride) {
        // Apply gravity to X position
        __m256 velocity = _mm256_load_ps(&velocities_x[i]);
        velocity = _mm256_add_ps(velocity, increment_x);
        _mm256_store_ps(&velocities_x[i], velocity);

        // Apply gravity to Y position
        velocity = _mm256_load_ps(&velocities_y[i]);
        velocity = _mm256_add_ps(velocity, increment_y);
        _mm256_store_ps(&velocities_y[i], velocity);
    }
}

void SimulationState::update_positions(float delta_time, const Vector2& bounds) {
    // Vectorize constants
    const __m256 dt = vsplat(delta_time);
    const __m256 bias = vsplat(DISTANCE_BIAS);
    const __m256 bounds_x = vsplat(bounds.x);
    const __m256 bounds_y = vsplat(bounds.y);

    constexpr int stride = VECTOR_BYTES / sizeof(float);
    for (int i = 0; i < visible_elements; i += stride) {
        // Apply velocity to X position
        __m256 position = _mm256_load_ps(&positions_x[i]);
        __m256 velocity = _mm256_load_ps(&velocities_x[i]);
        position = _mm256_fmadd_ps(velocity, dt, position);

        // Clamp position to horizontal bounds
        __m256 min = _mm256_load_ps(&radii[i]);
        min = _mm256_add_ps(min, bias);
        __m256 max = _mm256_sub_ps(bounds_x, min);
        position = _mm256_max_ps(position, min);
        position = _mm256_min_ps(position, max);

        _mm256_store_ps(&positions_x[i], position);

        // Apply velocity to Y position
        position = _mm256_load_ps(&positions_y[i]);
        velocity = _mm256_load_ps(&velocities_y[i]);
        position = _mm256_fmadd_ps(velocity, dt, position);

        // Clamp position to vertical bounds
        max = _mm256_sub_ps(bounds_y, min);
        position = _mm256_max_ps(position, min);
        position = _mm256_min_ps(position, max);

        _mm256_store_ps(&positions_y[i], position);
    }
}

void SimulationState::sweep_circle_to_circle() {
    constexpr int stride = VECTOR_BYTES / sizeof(float);
    for (int i = 0; i < sweep_elements.size(); i += stride) {
        // Sweeping a circle towards another circle is equivalent to raycasting from the sweep circle center towards a
        // circle with the combined radius of both circles at the target circle center

        const size_t* sweeps = &sweep_elements[i];
        const size_t* targets = &target_elements[i];

        // The offset vector of the target circle center relative to the sweep circle center
        __m256 offset_x = vdistribute(positions_x, targets);
        __m256 offset_y = vdistribute(positions_y, targets);
        __m256 temp = vdistribute(positions_x, sweeps);
        offset_x = _mm256_sub_ps(offset_x, temp);
        temp = vdistribute(positions_y, sweeps);
        offset_y = _mm256_sub_ps(offset_y, temp);

        // The absolute distance between the circle centers
        __m256 distance = vdot(offset_x, offset_y, offset_x, offset_y);
        distance = _mm256_sqrt_ps(distance);

        // The velocity of the sweep circle relative to the target circle
        __m256 relative_velocity_x = vdistribute(velocities_x, sweeps);
        __m256 relative_velocity_y = vdistribute(velocities_y, sweeps);
        temp = vdistribute(velocities_x, targets);
        relative_velocity_x = _mm256_sub_ps(relative_velocity_x, temp);
        temp = vdistribute(velocities_y, targets);
        relative_velocity_y = _mm256_sub_ps(relative_velocity_y, temp);

        // The absolute relative speed between the circle centers
        __m256 relative_speed =
            vdot(relative_velocity_x, relative_velocity_y, relative_velocity_x, relative_velocity_y);
        relative_speed = _mm256_sqrt_ps(relative_speed);

        // The combined radii of the circles
        __m256 total_radius = vdistribute(radii, sweeps);
        temp = vdistribute(radii, targets);
        total_radius = _mm256_add_ps(total_radius, temp);

        // The distance to the intersection with a line that is perpendicular to the relative velocity direction, and goes
        // through the target circle center
        __m256 a = vdot(offset_x, offset_y, relative_velocity_x, relative_velocity_y);
        a = _mm256_div_ps(a, relative_speed);

        // The distance from the target circle center to the intersection point with the line in the previous calculation
        temp = _mm256_mul_ps(distance, distance);
        temp = _mm256_fnmadd_ps(a, a, temp);
        temp = _mm256_sqrt_ps(temp);

        // The distance from the hit point to the intersection with the line in previous calculations
        __m256 b = _mm256_mul_ps(total_radius, total_radius);
        b = _mm256_fnmadd_ps(temp, temp, b);
        b = _mm256_sqrt_ps(b);

        // The time it takes for the ray to travel to the hit point with the inflated circle, or in other words, the time it
        // takes for the two circles to collide
        temp = _mm256_sub_ps(a, b);
        temp = _mm256_div_ps(temp, relative_speed);

        float hit_times[8];
        _mm256_storeu_ps(hit_times, temp);

        for (int j = 0; j < stride && i + j < sweep_elements.size(); j += 1) {
            const float hit_time = hit_times[j];
            if (hit_time > 0.0f && hit_time < collision_event.time_hit) {
                collision_event = {hit_time, sweeps[j], targets[j]};
            }
        }
    }
}

void SimulationState::sweep_circle_to_edge(EdgeTarget target, const Vector2& bounds, float delta_time) {
    Line line;
    switch (target) {
        case EdgeTarget::Left:
            line = {{0.0f, bounds.y}, {0.0f, -1.0f}};
            break;
        case EdgeTarget::Right:
            line = {{bounds.x, 0.0f}, {0.0f, 1.0f}};
            break;
        case EdgeTarget::Top:
            line = {{bounds.x, bounds.y}, {-1.0f, 0.0f}};
            break;
        case EdgeTarget::Bottom:
            line = {{0.0f, 0.0f}, {1.0f, 0.0f}};
            break;
    }

    sweep_elements.clear();

    for (size_t i = 0; i < visible_elements; i += 1) {
        const Vector2 position = {positions_x[i], positions_y[i]};
        const Vector2 velocity = {velocities_x[i], velocities_y[i]};
        const Vector2 half_movement = velocity * 0.5 * delta_time;
        const Vector2 bounding_position = position + half_movement;
        const float inflation = Vector2Length(half_movement);
        const Vector2 projected_position =
            line.point + line.direction * Vector2DotProduct(line.direction, bounding_position - line.point);
        const float bounding_distance_squared = Vector2LengthSqr(bounding_position - projected_position);
        const float contact_distance = radii[i] + inflation;

        if (bounding_distance_squared <= contact_distance * contact_distance) {
            sweep_elements.push_back(i);
        }
    }

    constexpr int stride = VECTOR_BYTES / sizeof(float);
    sweep_elements.resize((sweep_elements.size() + stride - 1) / stride * stride, 0);

    for (size_t i = 0; i < sweep_elements.size(); i += stride) {
        // Sweeping a circle to a line is equivalent to raycasting to an inflated line with the depth of two times the
        // circle radius. The calculcations below were derived by solving the following system of equations:
        //
        //   sweep_circle.center + hit_time * sweep_circle.velocity = target_line.point + u * target_line.direction
        //
        // By solving for hit_time, we get three cross products, that we can compute.

        const size_t* sweeps = &sweep_elements[i];

        __m256 line_direction_x = vsplat(line.direction.x);
        __m256 line_direction_y = vsplat(line.direction.y);

        // Calculate a point inflated by the circle radius from a normal of the line starting at the line point. Assume
        // circle will always be on the left side of the line direction.
        __m256 temp = vsplat(line.point.x);
        __m256 temp2 = vdistribute(radii, sweeps);
        __m256 temp3 = temp2;
        temp2 = _mm256_fnmadd_ps(line_direction_y, temp2, temp);
        temp = vsplat(line.point.y);
        temp3 = _mm256_fmadd_ps(line_direction_x, temp3, temp);

        // Cross product of the line direction and the inflated point
        temp = vcross(line_direction_x, line_direction_y, temp2, temp3);

        // Cross product of the circle center and the line direction
        temp2 = vdistribute(positions_x, sweeps);
        temp3 = vdistribute(positions_y, sweeps);
        temp2 = vcross(temp2, temp3, line_direction_x, line_direction_y);

        // Add the results together
        temp = _mm256_add_ps(temp, temp2);

        // Cross product of the line direction and the ray velocity vector
        temp2 = vdistribute(velocities_x, sweeps);
        temp3 = vdistribute(velocities_y, sweeps);
        temp2 = vcross(line_direction_x, line_direction_y, temp2, temp3);

        // Divide the previously added cross products with the last cross product to obtain the hit time
        temp = _mm256_div_ps(temp, temp2);

        float hit_times[8];
        _mm256_storeu_ps(hit_times, temp);

        for (size_t k = 0; k < stride && i + k < sweep_elements.size(); k += 1) {
            const float hit_time = hit_times[k];
            if (hit_time > 0.0f && hit_time < collision_event.time_hit) {
                collision_event = {hit_time, sweeps[k], (size_t)target};
            }
        }
    }
}

void SimulationState::copy_positions() {
    std::lock_guard<std::mutex> guard(copy_mutex);

    for (int i = 0; i < visible_elements; i += 1) {
        draw_positions[i].x = positions_x[i];
        draw_positions[i].y = positions_y[i];
    }
}
