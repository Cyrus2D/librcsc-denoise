#ifndef RCSC_PLAYER_PLAYER_POSITION_DENOISER_H
#define RCSC_PLAYER_PLAYER_POSITION_DENOISER_H

#include <rcsc/geom/vector_2d.h>

#include <array>
#include <functional>

namespace rcsc {

// Step 26 input. Angles are radians in the world model's team frame.
struct PlayerPositionInput {
    static constexpr int Window = 8;
    using Features = std::array<float, 16>;
    using Context = std::array<float, 35>;
    std::array<Features, Window> features{};
    std::array<Context, Window - 1> context{};
    int unum = 0;
    double low = 0, high = 0, bearing = 0, center = 0, raw_bearing = 0;
};

using PlayerPositionModel = std::function<Vector2D(const PlayerPositionInput &)>;

class PlayerPositionDenoiser {
public:
    struct Track {
        Vector2D pos{Vector2D::INVALIDATED};
        Vector2D vel{Vector2D::INVALIDATED};
        int age = 10;
    };
    using Tracks = std::array<Track, 23>;

    explicit PlayerPositionDenoiser(PlayerPositionModel model, bool apply);
    void reset();
    void begin(long cycle, const Vector2D & origin, double face, double focus_distance,
               double focus_direction, double speed_limit);
    Vector2D observe(int id, double distance, double bearing, double distance_change,
                     double direction_change, const Vector2D & stock, const Vector2D & velocity);
    void finish(const Tracks & tracks);
    long cycle() const { return M_cycle; }
    int age(int id) const { return M_history[id].age; }
    bool apply() const { return M_apply; }

private:
    struct History {
        long cycle = -1;
        int age = 0;
        Vector2D filtered{0, 0}, speed{0, 0}, visual{0, 0}, visual_speed{0, 0};
        std::array<double, 4> low{}, high{};
        double base = 0;
        PlayerPositionInput input;
        std::array<PlayerPositionInput::Context, PlayerPositionInput::Window> context{};
    };
    std::array<History, 23> M_history{};
    PlayerPositionModel M_model;
    bool M_apply;
    long M_cycle = -1;
    Vector2D M_origin{0, 0}, M_move{0, 0};
    double M_face = 0, M_focus_distance = 0, M_focus_direction = 0, M_speed_limit = 0;
};

} // namespace rcsc

#endif
