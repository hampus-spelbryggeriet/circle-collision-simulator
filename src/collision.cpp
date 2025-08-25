#include "collision.h"

#include <cmath>

#include "raylib.h"
#include "raymath.h"

float sweep_circle_to_circle(const Circle& sweep_circle, const Circle& target_circle) {
    const Vector2 offset = target_circle.center - sweep_circle.center;
    const Vector2 relative_velocity = sweep_circle.velocity - target_circle.velocity;
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

float sweep_circle_to_line(const Circle& sweep_circle, const Line& target_line) {
    // Assume circle will always be on the left side of the line direction.
    const Vector2 normal = {-target_line.direction.y, target_line.direction.x};

    const float time_hit = (Vector2CrossProduct(target_line.direction, target_line.point + normal * sweep_circle.radius)
                            + Vector2CrossProduct(sweep_circle.center, target_line.direction))
        / Vector2CrossProduct(target_line.direction, sweep_circle.velocity);

    return time_hit;
}
