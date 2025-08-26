#include "params.h"

#include <cstring>
#include <iostream>

void SimulationParameters::init_from_args(int argc, const char** argv) {
    *this = {};

    const char* key = nullptr;
    for (int i = 1; i < argc; i += 1) {
        if (key == nullptr) {
            key = argv[i];
            if (std::strcmp(key, "-h") == 0 || std::strcmp(key, "--help") == 0) {
                std::cout << "Usage: CircleCollisionSimulation [OPTION]..." << std::endl
                          << "Simulate colliding circles constrained within the bounds "
                             "of the window."
                          << std::endl
                          << std::endl
                          << "  -h, --help         Display this help and exit" << std::endl
                          << "  --width            Set the window width" << std::endl
                          << "  --height           Set the window height" << std::endl
                          << "  --spawn-limit      Set the maximum number of circles" << std::endl
                          << "  --spawn-rate       Set the circle spawn rate" << std::endl
                          << "  --min-radius       Set the minimum circle radius" << std::endl
                          << "  --max-radius       Set the maximum circle radius" << std::endl
                          << "  --simulation-speed Set the simulation speed. 1.0 is "
                             "full speed and 0.0 is completely still."
                          << std::endl
                          << "  --random-seed      Set the random seed" << std::endl;

                std::exit(0);
            }
        } else {
            const char* value = argv[i];
            if (strcmp(key, "--width") == 0) {
                width = parse_int(key, value);
            } else if (strcmp(key, "--height") == 0) {
                height = parse_int(key, value);
            } else if (strcmp(key, "--spawn-limit") == 0) {
                spawn_limit = parse_int(key, value);
            } else if (strcmp(key, "--spawn-rate") == 0) {
                spawn_rate = parse_int(key, value);
            } else if (strcmp(key, "--min-radius") == 0) {
                min_radius = parse_float(key, value);
            } else if (strcmp(key, "--max-radius") == 0) {
                max_radius = parse_float(key, value);
            } else if (strcmp(key, "--simulation-speed") == 0) {
                simulation_speed = parse_float(key, value);
            } else if (strcmp(key, "--random-seed") == 0) {
                random_seed = parse_int(key, value);
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

    if (width <= 0) {
        std::cerr << "Width must be positive" << std::endl;
        std::exit(1);
    }

    if (height <= 0) {
        std::cerr << "Height must be positive" << std::endl;
        std::exit(1);
    }

    if (spawn_limit <= 0) {
        std::cerr << "Spawn limit must be positive" << std::endl;
        std::exit(1);
    }

    if (spawn_rate <= 0.0f) {
        std::cerr << "Spawn rate must be positive" << std::endl;
        std::exit(1);
    }

    if (min_radius < 0.0f) {
        std::cerr << "Min radius must not be negative" << std::endl;
        std::exit(1);
    }

    if (max_radius < 0.0f) {
        std::cerr << "Max radius must not be negative" << std::endl;
        std::exit(1);
    }

    if (max_radius < min_radius) {
        std::cerr << "Max radius is smaller than min radius: " << max_radius << " < " << min_radius << std::endl;
        std::exit(1);
    }
}

int SimulationParameters::parse_int(const char* key, const char* value) {
    int n;

    try {
        n = std::stoi(value);
    } catch (const std::invalid_argument&) {
        std::cerr << "Invalid integer for argument `" << key << "`: " << value << std::endl;
        std::exit(1);
    } catch (const std::out_of_range&) {
        std::cerr << "Integer out of range for argument `" << key << "`: " << value << std::endl;
        std::exit(1);
    }

    return n;
}

float SimulationParameters::parse_float(const char* key, const char* value) {
    float f;

    try {
        f = std::stof(value);
    } catch (const std::invalid_argument&) {
        std::cerr << "Invalid float for argument `" << key << "`: " << value << std::endl;
        std::exit(1);
    } catch (const std::out_of_range&) {
        std::cerr << "Float out of range for argument `" << key << "`: " << value << std::endl;
        std::exit(1);
    }

    return f;
}
