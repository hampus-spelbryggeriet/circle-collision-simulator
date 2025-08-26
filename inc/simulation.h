#include <list>
#include <mutex>
#include <vector>

#include "params.h"
#include "raylib.h"

class SimulationState {
  public:
    SimulationState(int num_elements, float spawn_rate);
    ~SimulationState();

    int num_objects() {
        return visible_elements;
    }

    void init_random(const SimulationParameters& params);
    void update(float delta_time, const Vector2& gravity, const Vector2& bounds);
    void draw(const Vector2& bounds);

  private:
    struct CollisionEvent {
        float time_hit;
        size_t collider;
        size_t target;
    };

    enum class EdgeTarget {
        Left = -1,
        Right = -2,
        Top = -3,
        Bottom = -4,
    };

    static constexpr float TIME_BIAS = 1e-10f;
    static constexpr float DISTANCE_BIAS = 1e-10f;

    int num_elements;
    int visible_elements = 0;
    float* positions_x;
    float* positions_y;
    float* velocities_x;
    float* velocities_y;
    float* radii;
    float* masses;
    std::vector<int> markers;
    std::list<size_t> active_elements;
    std::vector<size_t> sweep_elements;
    std::vector<size_t> target_elements;
    CollisionEvent collision_event;

    Vector2* draw_positions;
    Color* colors;

    float spawn_rate;
    float spawn_timer;

    std::mutex copy_mutex;

    void insert_markers(size_t i, float delta_time);
    void insert_marker(size_t i, float key, float delta_time);
    void sort_markers(float delta_time);
    size_t calculate_marker_index(int marker);
    float calculate_marker_key(int marker, float delta_time);

    void update_velocities(float delta_time, const Vector2& gravity);
    void update_positions(float delta_time, const Vector2& bounds);
    void collide_circles();
    void collide_edge(EdgeTarget target, const Vector2& bounds, float delta_time);
    void copy_positions();
};
