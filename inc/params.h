#pragma once

#include <cstring>

class SimulationParameters {
  public:
    int width = 800;
    int height = 450;
    int spawn_limit = 300;
    float spawn_rate = 10.0;
    float min_radius = 5.0f;
    float max_radius = 10.0f;
    float simulation_speed = 1.0f;
    int random_seed = 0;

    void init_from_args(int argc, const char** argv);

  private:
    int parse_int(const char* key, const char* value);
    float parse_float(const char* key, const char* value);
};
