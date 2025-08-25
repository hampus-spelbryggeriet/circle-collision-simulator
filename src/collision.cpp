#include "collision.h"

#include <immintrin.h>

#include <cmath>

#include "raylib.h"
#include "raymath.h"
#include "simd.h"

void sweep_circle_to_circle(const Circle& sweep_circle, const CircleSimd& target_circle, float* hit_times) {
    // Sweeping a circle towards another circle is equivalent to raycasting from the sweep circle center towards a
    // circle with the combined radius of both circles at the target circle center

    // The offset vector of the target circle center relative to the sweep circle center
    __m256 offset_x = _mm256_loadu_ps(target_circle.centers_x);
    __m256 offset_y = _mm256_loadu_ps(target_circle.centers_y);
    __m256 temp = vsplat(sweep_circle.center.x);
    offset_x = _mm256_sub_ps(offset_x, temp);
    temp = vsplat(sweep_circle.center.y);
    offset_y = _mm256_sub_ps(offset_y, temp);

    // The absolute distance between the circle centers
    __m256 distance = vdot(offset_x, offset_y, offset_x, offset_y);
    distance = _mm256_sqrt_ps(distance);

    // The velocity of the sweep circle relative to the target circle
    __m256 relative_velocity_x = vsplat(sweep_circle.velocity.x);
    __m256 relative_velocity_y = vsplat(sweep_circle.velocity.y);
    temp = _mm256_loadu_ps(target_circle.velocities_x);
    relative_velocity_x = _mm256_sub_ps(relative_velocity_x, temp);
    temp = _mm256_loadu_ps(target_circle.velocities_y);
    relative_velocity_y = _mm256_sub_ps(relative_velocity_y, temp);

    // The absolute relative speed between the circle centers
    __m256 relative_speed = vdot(relative_velocity_x, relative_velocity_y, relative_velocity_x, relative_velocity_y);
    relative_speed = _mm256_sqrt_ps(relative_speed);

    // The combined radii of the circles
    __m256 total_radius = vsplat(sweep_circle.radius);
    temp = _mm256_loadu_ps(target_circle.radii);
    total_radius = _mm256_add_ps(total_radius, temp);

    // The distance to the intersection with a line that is perpendicular to the relative velocity direction, and goes
    // through the target circle center
    __m256 a = vdot(offset_x, offset_y, relative_velocity_x, relative_velocity_y);
    a = _mm256_div_ps(a, relative_speed);

    // The distance from the target circle center to the intersection point with the line in the previous calculation
    temp = _mm256_mul_ps(distance, distance);
    temp = _mm256_fnmadd_ps(a, a, temp);
    temp = _mm256_sqrt_ps(temp);

    // The distance from the hit point to the intersection with the line in previous calculations
    __m256 b = _mm256_mul_ps(total_radius, total_radius);
    b = _mm256_fnmadd_ps(temp, temp, b);
    b = _mm256_sqrt_ps(b);

    // The time it takes for the ray to travel to the hit point with the inflated circle, or in other words, the time it
    // takes for the two circles to collide
    temp = _mm256_sub_ps(a, b);
    temp = _mm256_div_ps(temp, relative_speed);

    _mm256_storeu_ps(hit_times, temp);
}

float sweep_circle_to_line(const CircleSimd& sweep_circle, const Line& target_line) {
    // Assume circle will always be on the left side of the line direction.
    const Vector2 normal = {-target_line.direction.y, target_line.direction.x};

    const float time_hit =
        (Vector2CrossProduct(target_line.direction, target_line.point + normal * sweep_circle.radii[0])
         + Vector2CrossProduct({sweep_circle.centers_x[0], sweep_circle.centers_y[0]}, target_line.direction))
        / Vector2CrossProduct(target_line.direction, {sweep_circle.velocities_x[0], sweep_circle.velocities_y[0]});

    return time_hit;
}
