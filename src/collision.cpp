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

void sweep_circle_to_line(const CircleSimd& sweep_circle, const Line& target_line, float* hit_times) {
    // Sweeping a circle to a line is equivalent to raycasting to an inflated line with the depth of two times the
    // circle radius. The calculcations below were derived by solving the following system of equations:
    //
    //   sweep_circle.center + hit_time * sweep_circle.velocity = target_line.point + u * target_line.direction
    //
    // By solving for hit_time, we get three cross products, that we can compute.

    __m256 line_direction_x = vsplat(target_line.direction.x);
    __m256 line_direction_y = vsplat(target_line.direction.y);

    // Calculate a point inflated by the circle radius from a normal of the line starting at the line point. Assume
    // circle will always be on the left side of the line direction.
    __m256 temp = vsplat(target_line.point.x);
    __m256 temp2 = _mm256_load_ps(sweep_circle.radii);
    __m256 temp3 = temp2;
    temp2 = _mm256_fnmadd_ps(line_direction_y, temp2, temp);
    temp = vsplat(target_line.point.y);
    temp3 = _mm256_fmadd_ps(line_direction_x, temp3, temp);

    // Cross product of the line direction and the inflated point
    temp = vcross(line_direction_x, line_direction_y, temp2, temp3);

    // Cross product of the circle center and the line direction
    temp2 = _mm256_load_ps(sweep_circle.centers_x);
    temp3 = _mm256_load_ps(sweep_circle.centers_y);
    temp2 = vcross(temp2, temp3, line_direction_x, line_direction_y);

    // Add the results together
    temp = _mm256_add_ps(temp, temp2);

    // Cross product of the line direction and the ray velocity vector
    temp2 = _mm256_load_ps(sweep_circle.velocities_x);
    temp3 = _mm256_load_ps(sweep_circle.velocities_y);
    temp2 = vcross(line_direction_x, line_direction_y, temp2, temp3);

    // Divide the previously added cross products with the last cross product to obtain the hit time
    temp = _mm256_div_ps(temp, temp2);

    _mm256_store_ps(hit_times, temp);
}
