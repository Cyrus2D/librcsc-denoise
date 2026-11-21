#include <rcsc/player/player_object.h>
#include <cassert>
#include <cmath>
int main()
{
    rcsc::Localization::PlayerT observation;observation.unum_=4;observation.pos_=rcsc::Vector2D(2,3);observation.rpos_=observation.pos_;
    rcsc::PlayerObject p(rcsc::LEFT,observation);p.update();p.update();p.update();
    const auto raw=p.pos(),seen=p.seenPos();const int age=p.posCount(),seen_age=p.seenPosCount();
    p.setPositionPrediction(rcsc::Vector2D(10,5),false);
    assert(p.pos().x==10&&p.trackedPos().x==raw.x&&p.posCount()==age);
    p.updateSelfBallRelated(rcsc::Vector2D(0,0),rcsc::Vector2D(10,5));assert(p.distFromBall()==0);
    assert(p.seenPos()==seen&&p.seenPosCount()==seen_age);
    p.clearPositionPrediction();assert(p.pos()==raw&&p.posCount()==age);
    p.setPositionPrediction(rcsc::Vector2D(10,5),true);
    assert(p.posCount()==0&&p.trackedPosCount()==age&&p.seenPosCount()==seen_age);
    p.clearPositionPrediction();p.update();assert(p.pos()==raw&&p.posCount()==age+1);
    // Exercise odd/even ages and clearing: inferred age never changes tracker/seen state.
    rcsc::PlayerObject q(rcsc::LEFT,observation);
    for ( int count = 1; count <= 30; ++count )
    {
        q.update();
        const auto tracked = q.trackedPos();
        const int seen_count = q.seenPosCount();
        assert(q.trackedPosCount()==count);
        q.setPositionPrediction(rcsc::Vector2D(10,5),rcsc::PlayerPredictionCountPolicy::Half);
        assert(q.posCount()==std::max(1,count/2));
        assert(q.trackedPosCount()==count && q.trackedPos()==tracked);
        assert(q.seenPosCount()==seen_count);
        q.clearPositionPrediction();
        assert(q.posCount()==count && q.pos()==tracked);
    }
}
