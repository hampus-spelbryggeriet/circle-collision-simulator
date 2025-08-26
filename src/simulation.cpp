#include "simulation.h"

#include <immintrin.h>
#include <sys/types.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <mutex>

#include "collision.h"
#include "raylib.h"
#include "raymath.h"
#include "simd.h"

#define VECTORIZATION 256
#define VECTOR_BYTES (VECTORIZATION / 8)

static float frand() {
    return float(std::rand()) / float(RAND_MAX);
}

template<typename T>
T* new_vector_array(size_t count) {
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
    if (spawn_timer < 0.0f) {
        visible_elements = std::min(visible_elements + 1, num_elements);
        while (spawn_timer < 0.0f) {
            spawn_timer += 1.0f / spawn_rate;
        }
    }

    update_velocities(delta_time, gravity);

    float remaining_time = delta_time;

    while (true) {
        collision_event.time_hit = remaining_time;

        collide_circles();
        collide_edge(EdgeTarget::Left, bounds);
        collide_edge(EdgeTarget::Right, bounds);
        collide_edge(EdgeTarget::Top, bounds);
        collide_edge(EdgeTarget::Bottom, bounds);

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

void SimulationState::collide_circles() {
    constexpr int stride = VECTOR_BYTES / sizeof(float);
    for (int i = 0; i < visible_elements; i += 1) {
        for (int j = i + 1; j < visible_elements; j += stride) {
            float hit_times[8];
            sweep_circle_to_circle(
                Circle {{positions_x[i], positions_y[i]}, {velocities_x[i], velocities_y[i]}, radii[i]},
                CircleSimd {&positions_x[j], &positions_y[j], &velocities_x[j], &velocities_y[j], &radii[j]},
                hit_times
            );

            for (int k = 0; k < stride && j + k < visible_elements; k += 1) {
                const float hit_time = hit_times[k];
                if (hit_time > 0.0f && hit_time < collision_event.time_hit) {
                    collision_event = {hit_time, i, j + k};
                }
            }
        }
    }
}

void SimulationState::collide_edge(EdgeTarget target, const Vector2& bounds) {
    Line line;
    switch (target) {
        case EdgeTarget::Left:
            line = {{0.0f, 0.0f}, {0.0f, -1.0f}};
            break;
        case EdgeTarget::Right:
            line = {{bounds.x, 0.0f}, {0.0f, 1.0f}};
            break;
        case EdgeTarget::Top:
            line = {{0.0f, bounds.y}, {-1.0f, 0.0f}};
            break;
        case EdgeTarget::Bottom:
            line = {{0.0f, 0.0f}, {1.0f, 0.0f}};
            break;
    }

    constexpr int stride = VECTOR_BYTES / sizeof(float);
    for (int i = 0; i < visible_elements; i += stride) {
        float hit_times[8];
        sweep_circle_to_line(
            CircleSimd {&positions_x[i], &positions_y[i], &velocities_x[i], &velocities_y[i], &radii[i]},
            line,
            hit_times
        );

        for (int k = 0; k < stride && i + k < visible_elements; k += 1) {
            const float hit_time = hit_times[k];
            if (hit_time > 0.0f && hit_time < collision_event.time_hit) {
                collision_event = {hit_time, i + k, (int)target};
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
