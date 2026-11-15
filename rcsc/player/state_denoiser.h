#ifndef RCSC_PLAYER_STATE_DENOISER_H
#define RCSC_PLAYER_STATE_DENOISER_H

#include <rcsc/geom/vector_2d.h>
#include <array>
#include <functional>
#include <utility>

namespace rcsc {

struct StateModelInput {
    static constexpr int Window = 8;
    std::array<std::array<float, 28>, Window> features{};
    std::array<std::array<float, 40>, Window> context{};
    std::array<int, Window> owner{};
    std::array<float, 8> baseline{};
    std::array<float, 3> geometry{};
    bool ball = false;
    bool controlled = false;
};
// A model candidate is independent of whether it is used for decisions.
struct StateDenoiseOptions {
    bool player_enabled = true, ball_enabled = true;
    bool apply_player = false, apply_ball = false;
    double player_velocity_min_dist = 10.0;
    double ball_position_min_dist = 5.0, ball_velocity_min_dist = 5.0;
};

using StateModel = std::function<std::array<float, 8>(const StateModelInput &)>;

class StateDenoiser {
public:
    struct Track {
        Vector2D pos{Vector2D::INVALIDATED}, vel{Vector2D::INVALIDATED};
        double body = 0;
        bool body_valid = false;
        int pos_age = 10, vel_age = 10, body_age = 10;
        double distance = 0, bearing = 0, dchange = 0, achange = 0;
        bool seen = false, motion = false, raw_body = false;
    };
    using Tracks = std::array<Track, 23>;
    explicit StateDenoiser(StateModel model);
    void reset();
    void update(long cycle, int self_id, bool right, const Vector2D & origin, const Vector2D & velocity,
                double face, double focus_dist, double focus_dir, double kickable_area,
                double ball_decay, bool self_kickable, const Tracks & tracks);
    bool ready(int id) const { return M_age[id] >= StateModelInput::Window; }
    const StateModelInput & input(int id) const { return M_input[id]; }
    std::array<float, 8> estimate(int id) const { return M_model(M_input[id]); }

private:
    StateModel M_model;
    std::array<StateModelInput, 23> M_input{};
    std::array<int, 23> M_age{};
    long M_cycle = -1;
};

// Shared inverse distance quantizer. Angles are radians.
std::pair<double, double> quantizedPlayerRange(double dist, double focus, double delta);

} // namespace rcsc
#endif
