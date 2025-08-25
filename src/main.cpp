#include <pthread.h>
#include <sched.h>
#include <sys/types.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <iostream>
#include <mutex>
#include <thread>

#include "params.h"
#include "raylib.h"
#include "simulation.h"

static void print_thread_info(const char* label) {
    static std::mutex mtx;

    std::lock_guard<std::mutex> guard(mtx);

    cpu_set_t cpuset;
    const int success = pthread_getaffinity_np(pthread_self(), sizeof(cpuset), &cpuset);
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
    SimulationState* state;
    Vector2 gravity;
    Vector2 bounds;
    std::atomic<float> elapsed;
};

static void* physics(void* ptr) {
    PhysicsContext* ctx = (PhysicsContext*)ptr;

    print_thread_info("Physics");

    SimulationState* state = ctx->state;
    const Vector2 gravity = ctx->gravity;
    const Vector2 bounds = ctx->bounds;

    constexpr float FPS = 120.f;
    constexpr float DELTA_TIME = 1.0f / 120.0f;

    while (!ctx->exit) {
        const clock_t frame_start = clock();
        state->update(DELTA_TIME, gravity, bounds);
        const clock_t update_end = clock();

        const long elapsed = 1000000000 * (update_end - frame_start) / CLOCKS_PER_SEC;
        std::this_thread::sleep_for(std::chrono::nanoseconds(long(1e9 * DELTA_TIME) - elapsed));

        ctx->elapsed = std::fmax(DELTA_TIME, elapsed / 1e9);
    }

    std::cout << "Exiting physics loop" << std::endl;

    return nullptr;
}

int main(int argc, const char** argv) {
    SimulationParameters params;
    params.init_from_args(argc, argv);

    srand(params.random_seed);

    InitWindow(params.width, params.height, "Code Test - Circle Collision Simulation");
    SetTargetFPS(60);

    Vector2 gravity = {0.0f, -9.82f * params.simulation_speed * params.simulation_speed};

    const int monitor = GetCurrentMonitor();
    const float physical_height = GetMonitorPhysicalHeight(monitor);
    if (physical_height > 0.0f) {
        gravity.y =
            1000.0f * -GetMonitorHeight(monitor) / physical_height * params.simulation_speed * params.simulation_speed;
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
    pthread_setaffinity_np(physics_thread, sizeof(physics_cpuset), &physics_cpuset);

    print_thread_info("Main");

    char main_fps_text[64];
    char main_ms_text[64];
    char physics_fps_text[64];
    char physics_ms_text[64];
    int main_fps_text_len = 0;
    int main_ms_text_len = 0;
    int physics_fps_text_len = 0;
    int physics_ms_text_len = 0;
    float fps_update_timer = -1.0f;

    while (!WindowShouldClose()) {
        const float delta_time = GetFrameTime();

        BeginDrawing();
        ClearBackground(BLACK);

        state.draw(bounds);

        {
            constexpr float FPS_UPDATE_RATE = 10.0f;
            constexpr float FPS_UPDATE_DELTA_TIME = 1.0f / FPS_UPDATE_RATE;

            constexpr int MAX_SEGMENTS_PER_ROW = 5;
            constexpr int MAX_ROWS = 3;
            constexpr int ROW_HEIGHT = 20;
            constexpr int FONT_SIZE = 16;
            constexpr int LABEL_WIDTH = 70;
            constexpr int FLOAT_WIDTH = 40;
            constexpr int UNIT_WIDTH = 20;
            constexpr int FLOAT_UNIT_GAP = 5;
            constexpr int OBJECTS_WIDTH = 2 * (FLOAT_WIDTH + FLOAT_UNIT_GAP + UNIT_WIDTH);

            if (fps_update_timer < 0.0f) {
                std::sprintf(main_fps_text, "%0.1f", 1.0f / delta_time);
                std::sprintf(main_ms_text, "%0.1f", 1000.0f * delta_time);
                std::sprintf(physics_fps_text, "%0.1f", 1.0f / physics_ctx.elapsed);
                std::sprintf(physics_ms_text, "%0.1f", 1000.0f * physics_ctx.elapsed);

                main_fps_text_len = MeasureText(main_fps_text, FONT_SIZE);
                main_ms_text_len = MeasureText(main_ms_text, FONT_SIZE);
                physics_fps_text_len = MeasureText(physics_fps_text, FONT_SIZE);
                physics_ms_text_len = MeasureText(physics_ms_text, FONT_SIZE);

                while (fps_update_timer < 0.0f) {
                    fps_update_timer += FPS_UPDATE_DELTA_TIME;
                }
            }

            char objects_text[64];
            sprintf(objects_text, "%i/%i", state.num_objects(), params.spawn_limit);
            const int objects_text_len = MeasureText(objects_text, FONT_SIZE);

            struct Segment {
                const char* text;
                int width;
            };

            const Segment segments[MAX_ROWS][MAX_SEGMENTS_PER_ROW] = {
                {{"Draw:", LABEL_WIDTH + FLOAT_WIDTH - main_fps_text_len},
                 {main_fps_text, main_fps_text_len + FLOAT_UNIT_GAP},
                 {"fps", UNIT_WIDTH + FLOAT_WIDTH - main_ms_text_len},
                 {main_ms_text, main_ms_text_len + FLOAT_UNIT_GAP},
                 {"ms", UNIT_WIDTH}},
                {{"Physics:", LABEL_WIDTH + FLOAT_WIDTH - physics_fps_text_len},
                 {physics_fps_text, physics_fps_text_len + FLOAT_UNIT_GAP},
                 {"fps", UNIT_WIDTH + FLOAT_WIDTH - physics_ms_text_len},
                 {physics_ms_text, physics_ms_text_len + FLOAT_UNIT_GAP},
                 {"ms", UNIT_WIDTH}},
                {{"Objects:", LABEL_WIDTH + OBJECTS_WIDTH - objects_text_len}, {objects_text, objects_text_len}},
            };

            int x = 0;
            int y = 0;

            for (int i = 0; i < MAX_ROWS; i += 1) {
                for (int j = 0; j < MAX_SEGMENTS_PER_ROW; j += 1) {
                    if (segments[i][j].text == nullptr) {
                        break;
                    }
                    DrawText(segments[i][j].text, x, y, FONT_SIZE, RAYWHITE);
                    x += segments[i][j].width;
                }
                y += ROW_HEIGHT;
                x = 0;
            }
        }

        EndDrawing();

        fps_update_timer -= delta_time;
    }

    physics_ctx.exit = true;
    pthread_join(physics_thread, nullptr);

    CloseWindow();

    return 0;
}
