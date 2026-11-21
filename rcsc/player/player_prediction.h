#ifndef RCSC_PLAYER_PLAYER_PREDICTION_H
#define RCSC_PLAYER_PLAYER_PREDICTION_H

#include <rcsc/geom/vector_2d.h>
#include <rcsc/types.h>
#include <functional>
#include <utility>
#include <string>
#include <vector>

namespace rcsc {
class WorldModel;

//! Age exposed to decision code while an accepted position overlay is active.
enum class PlayerPredictionCountPolicy : unsigned char {
    Preserve,
    Zero,
    Half //!< max(1, tracked posCount / 2), integer division.
};

//! Observation-only estimator output. Model loading/inference belongs to the client.
struct PlayerPositionPrediction {
    SideID side = NEUTRAL;
    int unum = 0;
    Vector2D stock{ Vector2D::INVALIDATED };
    Vector2D predicted{ Vector2D::INVALIDATED };
    int stock_count = 1000;
    int named_age = 0;
    double score = 0.0;
    bool eligible = false;
    bool selected = false;
    bool applied = false;
    int decision_count = 1000;
    std::vector< float > features; //!< Optional parity diagnostics; never truth.
};

struct PlayerPredictionFrame {
    long cycle = -1;
    int stopped = 0;
    bool active = false;
    double inference_us = 0.0;
    std::string status;
    std::vector< PlayerPositionPrediction > players;
};

using PlayerPredictionFunction = std::function< PlayerPredictionFrame( const WorldModel & ) >;
}
#endif
