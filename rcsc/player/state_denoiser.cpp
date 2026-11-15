#include "state_denoiser.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace rcsc {
namespace {
float halfValue(double value)
{
    int exponent;
    std::frexp(value, &exponent);
    const double step = std::ldexp(1., std::max(-24, exponent - 11));
    return std::nearbyint(value / step) * step;
}
}

StateDenoiser::StateDenoiser(StateModel model) : M_model(std::move(model)) {}

void StateDenoiser::reset()
{
    M_age.fill(0);
    M_cycle = -1;
}

void StateDenoiser::update(long cycle, int self_id, bool right, const Vector2D & origin,
                          const Vector2D & velocity, double face, double focus_dist, double focus_dir,
                          double kickable_area, double ball_decay, bool self_kickable, const Tracks & tracks)
{
    if (M_cycle < 0 || cycle != M_cycle + 1) reset();
    M_cycle = cycle;
    const auto team = [right](int id) { return id == 0 || ((id <= 11) != right); };
    const auto to_ball = [&](int id) {
        return tracks[id].pos.isValid() && tracks[0].pos.isValid()
            ? tracks[id].pos.dist(tracks[0].pos) : 150.;
    };
    double nearest = 150;
    int owner = 0;
    for (int id = 1; id < 23; ++id) {
        const double distance = to_ball(id);
        if (distance < nearest) { nearest = distance; owner = id; }
    }
    const bool controlled = nearest <= kickable_area || self_kickable;
    if (!controlled) owner = 0;
    if (right && owner) owner = (owner + 10) % 22 + 1;

    for (int id = 0; id < 23; ++id) {
        const Track & t = tracks[id];
        if (id == self_id || !t.seen || !t.pos.isValid()) { M_age[id] = 0; continue; }
        auto & input = M_input[id];
        if (M_age[id] == 0) input = StateModelInput{};
        M_age[id] = std::min(M_age[id] + 1, StateModelInput::Window);
        std::move(input.features.begin() + 1, input.features.end(), input.features.begin());
        std::move(input.context.begin() + 1, input.context.end(), input.context.begin());
        std::move(input.owner.begin() + 1, input.owner.end(), input.owner.begin());
        const bool has_vel = t.vel.isValid();
        const Vector2D vel = has_vel ? t.vel : Vector2D(0, 0);
        const double direction = t.body_valid ? t.body : std::atan2(vel.y, vel.x);
        const auto f = [](double x) { return static_cast<float>(x); };
        input.features.back() = {
            f(t.pos.x/55), f(t.pos.y/55), f(vel.x/2), f(vel.y/2),
            f(t.body_valid ? std::cos(direction) : 0), f(t.body_valid ? std::sin(direction) : 0),
            f(std::min(10, t.pos_age)/10.), f(has_vel ? std::min(10,t.vel_age)/10. : 0),
            f(t.body_valid ? std::min(10,t.body_age)/10. : 0),
            1, f(has_vel), f(t.body_valid), f(t.distance/60), f(std::cos(t.bearing)), f(std::sin(t.bearing)),
            f(t.motion ? t.dchange : 0), f(t.motion ? t.achange/10 : 0), f(t.motion), f(t.raw_body),
            f(origin.x/55), f(origin.y/55), f(velocity.x/2), f(velocity.y/2),
            f(std::cos(face)), f(std::sin(face)), f(team(id)), f(std::min(to_ball(id),30.)/30),
            f(std::min(nearest,30.)/30)
        };
        auto & context = input.context.back();
        context = {};
        std::array<int,4> closest{};
        std::array<double,4> distances;
        distances.fill(std::numeric_limits<double>::infinity());
        for (int j = 1; j < 23; ++j) {
            if (j == id || !tracks[j].pos.isValid()) continue;
            const int slot = team(id) == team(j) ? 0 : 2;
            const double d = tracks[j].pos.dist2(t.pos);
            if (d < distances[slot]) {
                distances[slot+1] = distances[slot]; closest[slot+1] = closest[slot];
                distances[slot] = d; closest[slot] = j;
            }
            else if (d < distances[slot+1]) { distances[slot+1] = d; closest[slot+1] = j; }
        }
        for (int slot = 0; slot < 4; ++slot) {
            float * out = context.data() + 10 * slot;
            out[6] = 1;
            if (!closest[slot]) continue;
            const Track & other = tracks[closest[slot]];
            const Vector2D offset = other.pos - t.pos;
            const bool relative_vel = has_vel && other.vel.isValid();
            const Vector2D diff = relative_vel ? other.vel - vel : Vector2D(0, 0);
            out[0] = halfValue(offset.x/20); out[1] = halfValue(offset.y/20);
            out[2] = halfValue(diff.x/2); out[3] = halfValue(diff.y/2);
            out[4] = other.body_valid ? halfValue(std::cos(other.body)) : 0;
            out[5] = other.body_valid ? halfValue(std::sin(other.body)) : 0;
            out[6] = halfValue(std::min(10,other.pos_age)/10.);
            out[7] = 1; out[8] = relative_vel; out[9] = other.body_valid;
        }
        input.owner.back() = id == 0 ? owner : 0;
        input.ball = id == 0;
        input.controlled = controlled;
        if (id == 0) {
            const Vector2D next = t.pos + vel;
            input.baseline = {f(t.pos.x), f(t.pos.y), f(vel.x), f(vel.y), f(next.x), f(next.y),
                              f(ball_decay*vel.x), f(ball_decay*vel.y)};
            const auto [low, high] = quantizedPlayerRange(t.distance, focus_dist, t.bearing - focus_dir);
            input.geometry = {f(low), f(high), 1};
        }
        else {
            input.baseline = {f(vel.x), f(vel.y), f(std::cos(direction)), f(std::sin(direction))};
        }
    }
}

} // namespace rcsc
