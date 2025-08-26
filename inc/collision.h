#pragma once

#include "raylib.h"

struct Line {
    Vector2 point;
    Vector2 direction;
};

struct Circle {
    Vector2 center;
    Vector2 velocity;
    float radius;
};

struct CircleSimd {
    const float* centers_x;
    const float* centers_y;
    const float* velocities_x;
    const float* velocities_y;
    const float* radii;
};

void sweep_circle_to_circle(const Circle& sweep_circle, const CircleSimd& target_circle, float* hit_times);
void sweep_circle_to_line(const CircleSimd& sweep_circle, const Line& target_line, float* hit_times);
