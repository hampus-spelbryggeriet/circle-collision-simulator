#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <sys/types.h>

#include "raylib.h"
#include "raymath.h"

struct SimulationParameters {
  int width = 800;
  int height = 450;
  int spawn_limit = 100;
  float spawn_rate = 1.0;
  float min_radius = 5.0f;
  float max_radius = 10.0f;
  float simulation_speed = 1.0f;
  int random_seed = 0;
};

int parse_int(const char *key, const char *value) {
  int n;

  try {
    n = std::stoi(value);
  } catch (const std::invalid_argument &) {
    std::cerr << "Invalid integer for argument `" << key << "`: " << value
              << std::endl;
    std::exit(1);
  } catch (const std::out_of_range &) {
    std::cerr << "Integer out of range for argument `" << key << "`: " << value
              << std::endl;
    std::exit(1);
  }

  return n;
}

float parse_float(const char *key, const char *value) {
  float f;

  try {
    f = std::stof(value);
  } catch (const std::invalid_argument &) {
    std::cerr << "Invalid float for argument `" << key << "`: " << value
              << std::endl;
    std::exit(1);
  } catch (const std::out_of_range &) {
    std::cerr << "Float out of range for argument `" << key << "`: " << value
              << std::endl;
    std::exit(1);
  }

  return f;
}

float frand() { return float(std::rand()) / float(RAND_MAX); }

SimulationParameters parse_args(int argc, const char **argv) {
  SimulationParameters params = {};

  const char *key = nullptr;
  for (int i = 1; i < argc; i += 1) {
    if (key == nullptr) {
      key = argv[i];
      if (strcmp(key, "-h") == 0 || strcmp(key, "--help") == 0) {
        std::cout << "Usage: CircleCollisionSimulation [OPTION]..." << std::endl
                  << "Simulate colliding circles constrained within the bounds "
                     "of the window."
                  << std::endl
                  << std::endl
                  << "  -h, --help         Display this help and exit"
                  << std::endl
                  << "  --width            Set the window width" << std::endl
                  << "  --height           Set the window height" << std::endl
                  << "  --spawn-limit      Set the maximum number of circles"
                  << "  --spawn-rate       Set the circle spawn rate"
                  << std::endl
                  << "  --min-radius       Set the minimum circle radius"
                  << std::endl
                  << "  --max-radius       Set the maximum circle radius"
                  << std::endl
                  << "  --simulation-speed Set the simulation speed. 1.0 is "
                     "full speed and 0.0 is completely still."
                  << std::endl
                  << "  --random-seed      Set the random seed" << std::endl;

        std::exit(0);
      }
    } else {
      const char *value = argv[i];
      if (strcmp(key, "--width") == 0) {
        params.width = parse_int(key, value);
      } else if (strcmp(key, "--height") == 0) {
        params.height = parse_int(key, value);
      } else if (strcmp(key, "--spawn-limit") == 0) {
        params.spawn_limit = parse_int(key, value);
      } else if (strcmp(key, "--spawn-rate") == 0) {
        params.spawn_rate = parse_int(key, value);
      } else if (strcmp(key, "--min-radius") == 0) {
        params.min_radius = parse_float(key, value);
      } else if (strcmp(key, "--max-radius") == 0) {
        params.max_radius = parse_float(key, value);
      } else if (strcmp(key, "--simulation-speed") == 0) {
        params.simulation_speed = parse_float(key, value);
      } else if (strcmp(key, "--random-seed") == 0) {
        params.random_seed = parse_int(key, value);
      } else {
        std::cerr << "Unepected argument: " << key << std::endl;
        std::exit(1);
      }
      key = nullptr;
    }
  }

  if (key != nullptr) {
    std::cerr << "Missing value: " << key << std::endl;
    std::exit(1);
  }

  if (params.width <= 0) {
    std::cerr << "Width must be positive" << std::endl;
    std::exit(1);
  }

  if (params.height <= 0) {
    std::cerr << "Height must be positive" << std::endl;
    std::exit(1);
  }

  if (params.spawn_limit <= 0) {
    std::cerr << "Spawn limit must be positive" << std::endl;
    std::exit(1);
  }

  if (params.spawn_rate <= 0.0f) {
    std::cerr << "Spawn rate must be positive" << std::endl;
    std::exit(1);
  }

  if (params.min_radius < 0.0f) {
    std::cerr << "Min radius must not be negative" << std::endl;
    std::exit(1);
  }

  if (params.max_radius < 0.0f) {
    std::cerr << "Max radius must not be negative" << std::endl;
    std::exit(1);
  }

  if (params.max_radius < params.min_radius) {
    std::cerr << "Max radius is smaller than min radius: " << params.max_radius
              << " < " << params.min_radius << std::endl;
    std::exit(1);
  }

  return params;
}

struct Line {
  Vector2 point;
  Vector2 direction;
};

struct Circle {
  Vector2 center;
  float radius;
  Vector2 velocity;
};

float sweep_circle_to_circle(const Circle &sweep_circle,
                             const Circle &target_circle) {
  const Vector2 offset = target_circle.center - sweep_circle.center;
  const Vector2 relative_velocity =
      sweep_circle.velocity - target_circle.velocity;
  const float relative_speed = Vector2Length(relative_velocity);
  const float total_radius = sweep_circle.radius + target_circle.radius;
  const float distance = Vector2Length(offset);
  const float a = Vector2DotProduct(offset, relative_velocity / relative_speed);
  const float b = std::sqrt(distance * distance - a * a);
  const float c = std::sqrt(total_radius * total_radius - b * b);
  const float distance_hit = a - c;
  const float time_hit = distance_hit / relative_speed;

  return time_hit;
}

float sweep_circle_to_line(const Circle &sweep_circle,
                           const Line &target_line) {
  // Assume circle will always be on the left side of the line direction.
  const Vector2 normal = {-target_line.direction.y, target_line.direction.x};

  const float time_hit =
      (Vector2CrossProduct(target_line.direction,
                           target_line.point + normal * sweep_circle.radius) +
       Vector2CrossProduct(sweep_circle.center, target_line.direction)) /
      Vector2CrossProduct(target_line.direction, sweep_circle.velocity);

  return time_hit;
}

struct SimulationState {
  SimulationState(int num_elements, float spawn_rate) {
    this->num_elements = num_elements;
    this->spawn_rate = spawn_rate;
    this->spawn_timer = 1.0f / spawn_rate;
    positions = new Vector2[num_elements];
    velocities = new Vector2[num_elements];
    radii = new float[num_elements];
    masses = new float[num_elements];
    colors = new Color[num_elements];
  }

  ~SimulationState() {
    delete[] colors;
    delete[] masses;
    delete[] radii;
    delete[] velocities;
    delete[] positions;
  }

  void initialize_random(const SimulationParameters &params) {
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

  void update(const Vector2 &gravity, const Vector2 &bounds) {
    if (spawn_timer < 0.0f) {
      visible_elements = std::min(visible_elements + 1, num_elements);
      while (spawn_timer < 0.0f) {
        spawn_timer += 1.0f / spawn_rate;
      }
    }

    update_velocities(gravity);

    const float delta_time = GetFrameTime();
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
          const Vector2 relative_velocity =
              velocities[collision_event.target] -
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

    spawn_timer -= delta_time;
  }

  void draw(const Vector2 &bounds) {
    for (int i = 0; i < visible_elements; i += 1) {
      DrawCircle(positions[i].x, bounds.y - positions[i].y, radii[i],
                 colors[i]);
    }
  }

private:
  struct CollisionEvent {
    float time_hit;
    int collider;
    int target;
  };

  enum class EdgeTarget {
    Left = -1,
    Right = -2,
    Top = -3,
    Bottom = -4,
  };

  static constexpr float TIME_BIAS = 1e-10f;

  int num_elements;
  int visible_elements = 0;
  Vector2 *positions;
  Vector2 *velocities;
  float *radii;
  float *masses;
  Color *colors;
  CollisionEvent collision_event;

  float spawn_rate;
  float spawn_timer;

  void update_velocities(const Vector2 &gravity) {
    const float delta_time = GetFrameTime();

    const Vector2 increment = gravity * delta_time;
    for (int i = 0; i < visible_elements; i += 1) {
      velocities[i] += increment;
    }
  }

  void update_positions(float delta_time, const Vector2 &bounds) {
    for (int i = 0; i < visible_elements; i += 1) {
      positions[i] += velocities[i] * delta_time;
      positions[i] = Vector2Clamp(positions[i], Vector2Ones * radii[i],
                                  bounds - Vector2Ones * radii[i]);
    }
  }

  void collide_circles() {
    for (int i = 0; i < visible_elements; i += 1) {
      for (int j = i + 1; j < visible_elements; j += 1) {
        const float time_hit = sweep_circle_to_circle(
            Circle{positions[i], radii[i], velocities[i]},
            Circle{positions[j], radii[j], velocities[j]});

        if (time_hit > 0.0f && time_hit < collision_event.time_hit) {
          collision_event = {time_hit, i, j};
        }
      }
    }
  }

  void collide_edge(EdgeTarget target, const Vector2 &bounds) {
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
};

int main(int argc, const char **argv) {
  SimulationParameters params = parse_args(argc, argv);

  srand(params.random_seed);

  InitWindow(params.width, params.height,
             "Code Test - Circle Collision Simulation");
  SetTargetFPS(60);

  Vector2 gravity = {0.0f, -9.82f * params.simulation_speed *
                               params.simulation_speed};

  const int monitor = GetCurrentMonitor();
  const float physical_height = GetMonitorPhysicalHeight(monitor);
  if (physical_height > 0.0f) {
    gravity.y = 1000.0f * -GetMonitorHeight(monitor) / physical_height *
                params.simulation_speed * params.simulation_speed;
  }

  SimulationState state(params.spawn_limit, params.spawn_rate);
  state.initialize_random(params);

  const Vector2 bounds = {float(params.width), float(params.height)};

  while (!WindowShouldClose()) {
    state.update(gravity, bounds);

    BeginDrawing();
    ClearBackground(BLACK);

    state.draw(bounds);

    EndDrawing();
  }

  CloseWindow();

  return 0;
}
