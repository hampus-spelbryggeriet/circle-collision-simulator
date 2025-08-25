#pragma once

#include "raylib.h"

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
                             const Circle &target_circle);
float sweep_circle_to_line(const Circle &sweep_circle, const Line &target_line);
