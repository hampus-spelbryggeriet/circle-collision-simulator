#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <mutex>
#include <sys/types.h>

#include "raylib.h"
#include "raymath.h"

#include "collision.h"
#include "simulation.h"

static float frand() { return float(std::rand()) / float(RAND_MAX); }

SimulationState::SimulationState(int num_elements, float spawn_rate) {
  this->num_elements = num_elements;
  this->spawn_rate = spawn_rate;
  this->spawn_timer = 1.0f / spawn_rate;
  positions = new Vector2[num_elements];
  velocities = new Vector2[num_elements];
  radii = new float[num_elements];
  masses = new float[num_elements];
  draw_positions = new Vector2[num_elements];
  colors = new Color[num_elements];
}

SimulationState::~SimulationState() {
  delete[] draw_positions;
  delete[] colors;
  delete[] masses;
  delete[] radii;
  delete[] velocities;
  delete[] positions;
}

void SimulationState::init_random(const SimulationParameters &params) {
  for (int i = 0; i < num_elements; i += 1) {
    radii[i] =
        params.min_radius + (params.max_radius - params.min_radius) * frand();
    masses[i] = PI * radii[i] * radii[i];
    positions[i] = {radii[i] + (params.width - 2.0f * radii[i]) * frand(),
                    radii[i] + (params.height - 2.0f * radii[i]) * frand()};
    velocities[i] = Vector2Zeros;
    colors[i] = {uint8_t(128 + 127 * frand()), uint8_t(128 + 127 * frand()),
                 uint8_t(128 + 127 * frand()), 255};
  }
}

void SimulationState::update(float delta_time, const Vector2 &gravity,
                             const Vector2 &bounds) {
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
        velocities[collision_event.collider].x *= -1.0f;
        break;
      case EdgeTarget::Top:
      case EdgeTarget::Bottom:
        velocities[collision_event.collider].y *= -1.0f;
        break;
      default:
        const Vector2 relative_velocity = velocities[collision_event.target] -
                                          velocities[collision_event.collider];
        const Vector2 offset_hit = positions[collision_event.target] -
                                   positions[collision_event.collider];
        const float distance_hit = Vector2Length(offset_hit);
        const Vector2 reflect_normal =
            offset_hit / (distance_hit * distance_hit);
        const float total_mass =
            masses[collision_event.collider] + masses[collision_event.target];
        const float k = 2.0f *
                        Vector2DotProduct(relative_velocity, offset_hit) /
                        total_mass;

        velocities[collision_event.collider] +=
            reflect_normal * k * masses[collision_event.target];
        velocities[collision_event.target] -=
            reflect_normal * k * masses[collision_event.collider];
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

void SimulationState::draw(const Vector2 &bounds) {
  std::lock_guard<std::mutex> guard(copy_mutex);

  for (int i = 0; i < visible_elements; i += 1) {
    DrawCircle(draw_positions[i].x, bounds.y - draw_positions[i].y, radii[i],
               colors[i]);
  }
}

void SimulationState::update_velocities(float delta_time,
                                        const Vector2 &gravity) {
  const Vector2 increment = gravity * delta_time;
  for (int i = 0; i < visible_elements; i += 1) {
    velocities[i] += increment;
  }
}

void SimulationState::update_positions(float delta_time,
                                       const Vector2 &bounds) {
  for (int i = 0; i < visible_elements; i += 1) {
    positions[i] += velocities[i] * delta_time;

    const Vector2 min = Vector2Ones * (radii[i] + DISTANCE_BIAS);
    const Vector2 max = bounds - min;
    positions[i] = Vector2Clamp(positions[i], min, max);
  }
}

void SimulationState::collide_circles() {
  for (int i = 0; i < visible_elements; i += 1) {
    for (int j = i + 1; j < visible_elements; j += 1) {
      const float time_hit =
          sweep_circle_to_circle(Circle{positions[i], radii[i], velocities[i]},
                                 Circle{positions[j], radii[j], velocities[j]});

      if (time_hit > 0.0f && time_hit < collision_event.time_hit) {
        collision_event = {time_hit, i, j};
      }
    }
  }
}

void SimulationState::collide_edge(EdgeTarget target, const Vector2 &bounds) {
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

  for (int i = 0; i < visible_elements; i += 1) {
    const float time_hit = sweep_circle_to_line(
        Circle{positions[i], radii[i], velocities[i]}, line);
    if (time_hit > 0.0f && time_hit < collision_event.time_hit) {
      collision_event = {time_hit, i, (int)target};
    }
  }
}

void SimulationState::copy_positions() {
  std::lock_guard<std::mutex> guard(copy_mutex);

  for (int i = 0; i < visible_elements; i += 1) {
    draw_positions[i] = positions[i];
  }
}
