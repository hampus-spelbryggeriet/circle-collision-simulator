#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <iostream>
#include <mutex>
#include <pthread.h>
#include <sched.h>
#include <sys/types.h>
#include <thread>

#include "raylib.h"

#include "params.h"
#include "simulation.h"

static void print_thread_info(const char *label) {
  static std::mutex mtx;

  std::lock_guard<std::mutex> guard(mtx);

  cpu_set_t cpuset;
  const int success =
      pthread_getaffinity_np(pthread_self(), sizeof(cpuset), &cpuset);
  if (success != 0) {
    std::cerr << "Failed to get affinity info" << std::endl;
    std::exit(1);
  }

  std::cout << label << " thread running on core ";
  for (size_t i = 0; i < CPU_SETSIZE; i += 1) {
    if (CPU_ISSET(i, &cpuset)) {
      std::cout << i << std::endl;
      return;
    }
  }
}

struct PhysicsContext {
  std::atomic_bool exit;
  SimulationState *state;
  Vector2 gravity;
  Vector2 bounds;
};

static void *physics(void *ptr) {
  PhysicsContext *ctx = (PhysicsContext *)ptr;

  print_thread_info("Physics");

  SimulationState *state = ctx->state;
  const Vector2 gravity = ctx->gravity;
  const Vector2 bounds = ctx->bounds;

  constexpr float FPS = 120.f;
  constexpr float DELTA_TIME = 1.0f / 120.0f;

  while (!ctx->exit) {
    const clock_t frame_start = clock();
    state->update(DELTA_TIME, gravity, bounds);
    const clock_t frame_end = clock();

    const long elapsed =
        1000000000 * (frame_end - frame_start) / CLOCKS_PER_SEC;
    std::this_thread::sleep_for(
        std::chrono::nanoseconds(long(1e9 * DELTA_TIME) - elapsed));
  }

  std::cout << "Exiting physics loop" << std::endl;

  return nullptr;
}

int main(int argc, const char **argv) {
  SimulationParameters params;
  params.init_from_args(argc, argv);

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
  state.init_random(params);

  const Vector2 bounds = {float(params.width), float(params.height)};

  pthread_t main_thread = pthread_self();
  cpu_set_t main_cpuset;
  CPU_ZERO(&main_cpuset);
  CPU_SET(0, &main_cpuset);
  pthread_setaffinity_np(main_thread, sizeof(main_cpuset), &main_cpuset);

  PhysicsContext physics_ctx = {
      .exit = {false},
      .state = &state,
      .gravity = gravity,
      .bounds = bounds,
  };

  pthread_t physics_thread;
  cpu_set_t physics_cpuset;
  CPU_ZERO(&physics_cpuset);
  CPU_SET(1, &physics_cpuset);
  pthread_create(&physics_thread, nullptr, physics, &physics_ctx);
  pthread_setaffinity_np(physics_thread, sizeof(physics_cpuset),
                         &physics_cpuset);

  print_thread_info("Main");

  while (!WindowShouldClose()) {
    BeginDrawing();
    ClearBackground(BLACK);

    state.draw(bounds);

    EndDrawing();
  }

  physics_ctx.exit.store(true);
  pthread_join(physics_thread, nullptr);

  CloseWindow();

  return 0;
}
