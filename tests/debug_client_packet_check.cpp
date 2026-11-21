#include <rcsc/player/player_agent.h>
#include <rcsc/player/debug_client.h>
#include <rcsc/geom/rect_2d.h>
#include <cstdlib>
#include <string>

class ProbeAgent : public rcsc::PlayerAgent {
    void actionImpl() override {}
};

int main( int argc, char ** argv )
{
    if ( argc != 2 ) return 2;
    ProbeAgent agent;
    auto & dc = agent.debugClient();
    dc.connect( "127.0.0.1", std::atoi( argv[1] ) );
    dc.setMaxPacketSize( 7800 );
    dc.setMessage( std::string( 16000, 'X' ) );
    for ( int i = 0; i < 50; ++i )
    {
        const rcsc::Vector2D p( i + .123, i + .456 );
        dc.addCircle( p, .3, "#ff9f1c" );
        dc.addRectangle( rcsc::Rect2D::from_center( p, .6, .6 ), "#00d5ff" );
        dc.addLine( p, p + rcsc::Vector2D( 3, 4 ), "#00d5ff" );
        dc.addTriangle( p, p + rcsc::Vector2D( 3, 4 ), p + rcsc::Vector2D( 5, 6 ), "#00d5ff" );
    }
    dc.writeAll( agent.world(), agent.effector() );

    dc.setMessage( "packet-check" );
    dc.addCircle( rcsc::Vector2D( 1, 2 ), .3, "#ff9f1c" );
    dc.addRectangle( rcsc::Rect2D::from_center( rcsc::Vector2D( 3, 4 ), .6, .6 ), "#00d5ff" );
    dc.addLine( rcsc::Vector2D( 1, 2 ), rcsc::Vector2D( 3, 4 ), "#00d5ff" );
    dc.writeAll( agent.world(), agent.effector() );

    dc.addCircle( rcsc::Vector2D( 99, 99 ), 3, "red" );
    dc.setTarget( rcsc::Vector2D( 99, 99 ) );
    dc.clearDrawings();
    dc.setMessage( "cleared-check" );
    dc.addCircle( rcsc::Vector2D( 5, 6 ), .3, "#ff9f1c" );
    dc.writeAll( agent.world(), agent.effector() );
}
