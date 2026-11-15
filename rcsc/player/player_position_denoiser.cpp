#include "player_position_denoiser.h"
#include "state_denoiser.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <utility>

namespace rcsc {
namespace {
constexpr double Pi = 3.14159265358979323846;
constexpr double HalfStep = Pi / 360;
constexpr double Diagonal = .7071067811865475244;
const std::array<Vector2D, 4> Axes{{{1, 0}, {0, 1}, {Diagonal, Diagonal}, {Diagonal, -Diagonal}}};

double dot(const Vector2D & a, const Vector2D & b)
{
    return a.x * b.x + a.y * b.y;
}

Vector2D ray(double radians)
{
    return Vector2D(std::cos(radians), std::sin(radians));
}

Vector2D cap(const Vector2D & velocity)
{
    return velocity / std::max(1., velocity.r() / 1.5);
}

} // namespace

std::pair<double, double> quantizedPlayerRange(double dist, double focus, double delta)
{
    const int observed = static_cast<int>(std::nearbyint(dist * 10));
    if (focus == 0) {
        static const auto bins = [] {
            std::map<int, std::pair<double, double>> result;
            for (int k = -230; k < 65; ++k) {
                const int key = static_cast<int>(std::nearbyint(10 * std::exp(k / 10.)));
                const double lo = std::max(0., std::exp((k - .5) / 10) - 1e-10);
                const double hi = std::max(0., std::exp((k + .5) / 10) - 1e-10);
                const auto found = result.find(key);
                result[key] = {found == result.end() ? lo : found->second.first, hi};
            }
            result[0].first = 0;
            return result;
        }();
        const auto found = bins.find(observed);
        if (found != bins.end()) return found->second;
    }
    else {
        const double radius = .065 * (dist + focus) + .15;
        const double cos_delta = std::cos(delta);
        double low = std::numeric_limits<double>::infinity(), high = -low;
        const auto quantize = [](double x) {
            return std::exp(std::nearbyint(std::log(x + 1e-10) * 10) / 10);
        };
        for (int i = 0; i <= 80; ++i) {
            const double x = std::max(0., dist + radius * (i / 40. - 1));
            const double f = std::sqrt(std::max(0., x * x + focus * focus - 2 * x * focus * cos_delta));
            const double value = std::max(0., (x - f + quantize(x) + quantize(f)) / 2);
            if (std::nearbyint(value * 10) == observed) {
                low = std::min(low, x);
                high = std::max(high, x);
            }
        }
        if (low <= high) return {low, high};
    }
    return {std::max(0., .94 * dist - .05), 1.06 * dist + .05};
}

namespace {
// The training sidecars store context as binary16.
float contextValue(double value)
{
    int exponent;
    std::frexp(value, &exponent);
    const double step = std::ldexp(1., std::max(-24, exponent - 11));
    return static_cast<float>(std::nearbyint(value / step) * step);
}
} // namespace

PlayerPositionDenoiser::PlayerPositionDenoiser(PlayerPositionModel model, bool apply)
    : M_model(std::move(model)), M_apply(apply)
{
}

void PlayerPositionDenoiser::reset()
{
    M_history = {};
    M_cycle = -1;
}

void PlayerPositionDenoiser::begin(long cycle, const Vector2D & origin, double face,
                                  double focus_distance, double focus_direction, double speed_limit)
{
    if (M_cycle < 0 || cycle != M_cycle + 1) reset();
    M_move = M_cycle < 0 ? Vector2D(0, 0) : origin - M_origin;
    M_cycle = cycle;
    M_origin = origin;
    M_face = face;
    M_focus_distance = focus_distance;
    M_focus_direction = focus_direction;
    M_speed_limit = speed_limit;
}

Vector2D PlayerPositionDenoiser::observe(int id, double distance, double bearing,
                                        double distance_change, double direction_change,
                                        const Vector2D & stock, const Vector2D & velocity)
{
    History & h = M_history[id];
    if (h.cycle == M_cycle) return Vector2D(Vector2D::INVALIDATED);
    const bool continuing = h.cycle >= 0 && h.cycle + 1 == M_cycle;
    if (!continuing) h = History{};
    h.cycle = M_cycle;
    h.age = std::min(h.age + 1, 10);

    const double angle = M_face + bearing;
    const auto [lo, hi] = quantizedPlayerRange(distance, M_focus_distance, bearing - M_focus_direction);
    const Vector2D direction = ray(angle), tangent(-direction.y, direction.x);
    const Vector2D prior = continuing ? h.filtered + h.speed : stock;
    const double radial_prior = dot(prior - M_origin, direction);
    const double center = (lo + hi) / 2, width = hi - lo;
    const double estimate = continuing
        ? center + width * width / (width * width + .18 * .18 * 12)
          * (std::clamp(radial_prior, lo, hi) - center)
        : center;
    const Vector2D position = M_origin + direction * estimate;
    const bool has_velocity = velocity.isValid();
    const Vector2D measured = has_velocity ? velocity : (continuing ? position - h.filtered : Vector2D(0, 0));
    const Vector2D visual_motion = has_velocity ? velocity : (continuing ? stock - h.visual : Vector2D(0, 0));
    const Vector2D cv = continuing ? h.visual + h.visual_speed : stock;
    const Vector2D blend = stock * .65 + cv * .35 - M_origin;
    h.speed = cap(continuing ? h.speed * .65 + measured * .35 : measured);
    h.visual_speed = cap(continuing ? h.visual_speed * .65 + visual_motion * .35 : visual_motion);
    h.filtered = position;
    h.visual = stock;

    const std::array<double, 16> features{
        distance / 60, std::sin(bearing), std::cos(bearing), dot(direction, M_move), dot(tangent, M_move),
        has_velocity ? dot(velocity, direction) : 0, has_velocity ? dot(velocity, tangent) : 0,
        has_velocity ? 1. : 0., distance_change, direction_change / 10,
        (estimate - center) / std::max(.01, width), width / 3,
        std::clamp(radial_prior - center, -8., 8.) / 3, h.age / 10.,
        M_focus_distance / 10, std::sin(bearing - M_focus_direction)
    };
    auto & input = h.input;
    std::move(input.features.begin() + 1, input.features.end(), input.features.begin());
    std::transform(features.begin(), features.end(), input.features.back().begin(),
                   [](double x) { return static_cast<float>(x); });
    std::move(h.context.begin() + 1, h.context.end(), h.context.begin());
    for (int i = 0; i < PlayerPositionInput::Window - 1; ++i) input.context[i] = h.context[i];
    h.context.back() = {};

    // Expand the previous feasible region along the field axes and diagonals.
    const double base = std::clamp(blend.r(), lo, hi);
    const double turn = std::remainder(std::atan2(blend.y, blend.x) - angle, 2 * Pi);
    const double bounded_angle = angle + std::clamp(turn, -HalfStep, HalfStep);
    const double reach = M_speed_limit + .12 + .0044 * (h.base + base);
    h.base = base;
    const Vector2D preferred = ray(bounded_angle) * base;
    struct Candidate { double angle, lo, hi; Vector2D ray; };
    std::array<Candidate, 6> candidates{};
    bool feasible = false;
    for (int i = 0; i < 6; ++i) {
        auto & c = candidates[i];
        c = {i < 5 ? angle + (i - 2) * HalfStep / 2 : bounded_angle, lo, hi, {}};
        c.ray = ray(c.angle);
        if (continuing) {
            for (int j = 0; j < 4; ++j) {
                const double projection = dot(c.ray, Axes[j]);
                const double origin = dot(M_origin, Axes[j]);
                const double low = h.low[j] - reach - origin, high = h.high[j] + reach - origin;
                if (std::abs(projection) < 1e-7) {
                    if (low > 0 || high < 0) c.hi = -1;
                }
                else {
                    const double a = low / projection, b = high / projection;
                    c.lo = std::max(c.lo, std::min(a, b));
                    c.hi = std::min(c.hi, std::max(a, b));
                }
            }
        }
        feasible |= c.lo <= c.hi;
    }
    h.low.fill(std::numeric_limits<double>::infinity());
    h.high.fill(-std::numeric_limits<double>::infinity());
    double best = std::numeric_limits<double>::infinity();
    const double padding = hi * HalfStep / 4;
    for (auto & c : candidates) {
        if (!feasible) { c.lo = lo; c.hi = hi; }
        if (c.lo > c.hi) continue;
        const double radius = std::clamp(dot(c.ray, preferred), c.lo, c.hi);
        const double cost = (c.ray * radius - preferred).r2();
        if (cost < best) {
            best = cost;
            input.low = c.lo;
            input.high = c.hi;
            input.bearing = c.angle;
            input.center = radius;
        }
        for (int j = 0; j < 4; ++j) {
            const double projection = dot(c.ray, Axes[j]);
            const double origin = dot(M_origin, Axes[j]);
            h.low[j] = std::min(h.low[j], origin + std::min(c.lo * projection, c.hi * projection) - padding);
            h.high[j] = std::max(h.high[j], origin + std::max(c.lo * projection, c.hi * projection) + padding);
        }
    }
    input.raw_bearing = angle;
    input.unum = (id - 1) % 11 + 1;
    if (h.age < PlayerPositionInput::Window) return Vector2D(Vector2D::INVALIDATED);
    const Vector2D correction = M_model(input);
    if (!correction.isValid() || !std::isfinite(correction.x) || !std::isfinite(correction.y))
        return Vector2D(Vector2D::INVALIDATED);
    return M_origin + correction;
}

void PlayerPositionDenoiser::finish(const Tracks & tracks)
{
    const Vector2D forward = ray(M_face), lateral(-forward.y, forward.x);
    for (int id = 1; id < 23; ++id) {
        if (M_history[id].cycle != M_cycle) continue;
        const Track & target = tracks[id];
        std::array<int, 5> selected{0, -1, -1, -1, -1};
        std::array<double, 5> distances{};
        distances.fill(std::numeric_limits<double>::infinity());
        if (target.pos.isValid()) {
            for (int other = 1; other < 23; ++other) {
                if (other == id || !tracks[other].pos.isValid()) continue;
                const int first = ((id <= 11) == (other <= 11)) ? 1 : 3;
                const double dist = (tracks[other].pos - target.pos).r2();
                if (dist < distances[first]) {
                    distances[first + 1] = distances[first];
                    selected[first + 1] = selected[first];
                    distances[first] = dist;
                    selected[first] = other;
                }
                else if (dist < distances[first + 1]) {
                    distances[first + 1] = dist;
                    selected[first + 1] = other;
                }
            }
        }
        auto & result = M_history[id].context.back();
        for (int slot = 0; slot < 5; ++slot) {
            const int index = selected[slot];
            float * out = result.data() + slot * 7;
            out[4] = 1;
            if (index < 0 || !target.pos.isValid() || !tracks[index].pos.isValid()) continue;
            const Track & other = tracks[index];
            const Vector2D pos = other.pos - target.pos;
            out[0] = contextValue(dot(pos, forward) / 20);
            out[1] = contextValue(dot(pos, lateral) / 20);
            out[4] = contextValue(std::min(10, other.age) / 10.);
            out[5] = 1;
            if (target.vel.isValid() && other.vel.isValid()) {
                const Vector2D vel = other.vel - target.vel;
                out[2] = contextValue(dot(vel, forward) / 2);
                out[3] = contextValue(dot(vel, lateral) / 2);
                out[6] = 1;
            }
        }
    }
}

} // namespace rcsc
