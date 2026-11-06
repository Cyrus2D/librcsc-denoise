// -*-c++-*-

/*!
  \file denoise_logger.cpp
  \brief per-agent dataset logger for training observation denoisers. Source File
*/

/*
 *Copyright:

 Copyright (C) Hidehisa AKIYAMA

 This code is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 3 of the License, or (at your option) any later version.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

 *EndCopyright:
 */

/////////////////////////////////////////////////////////////////////

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include "denoise_logger.h"

#include "player_agent.h"
#include "action_effector.h"
#include "ball_object.h"
#include "body_sensor.h"
#include "fullstate_sensor.h"
#include "object_table.h"
#include "player_config.h"
#include "player_object.h"
#include "see_state.h"
#include "self_object.h"
#include "view_mode.h"
#include "visual_sensor.h"
#include "world_model.h"

#include <rcsc/common/audio_memory.h>
#include <rcsc/common/player_param.h>
#include <rcsc/common/player_type.h>
#include <rcsc/common/server_param.h>
#include <rcsc/geom/angle_deg.h>
#include <rcsc/geom/vector_2d.h>
#include <rcsc/game_mode.h>
#include <rcsc/types.h>

#ifdef HAVE_LIBZ
#include <rcsc/gz/gzfstream.h>
#endif

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <sys/stat.h>
#include <sys/types.h>
#ifdef _WIN32
#include <direct.h>
#endif

namespace rcsc {

const int DenoiseLogger::SCHEMA_VERSION = 2;

namespace {

//! number of decimals of the normal numeric columns
const int PREC = 4;
//! number of decimals of the ground truth geometry columns
const int PREC_TRUE = 6;

//! number of cycles between two flushes
const int FLUSH_INTERVAL = 50;

//! max number of placeholder rows written for one jump of the cycle number
const int MAX_PLACEHOLDER_ROWS = 6000;

//! max number of raw stream rows kept before the output directory is known
const std::size_t MAX_RAW_PENDING = 4000;

// ---------------------------------------------------------------------------
// column definitions. keep the order. See .github/instructions/denoise-dataset.instructions.md
// ---------------------------------------------------------------------------

#define DN_KEYS "game_id,side,unum,cycle,stopped"

const char * const COLS_AGENT_CYCLE =
    DN_KEYS ","
    // 1 = a decision was made in this cycle, 0 = placeholder row of a skipped cycle
    "decision_made,"
    // game state and timing
    "game_mode,score_l,score_r,"
    "sb_received,see_received,see_used,see_count_this_cycle,fullstate_received,truth_valid,"
    "decision_trigger,msec_sb_to_decision,"
    // view
    "view_width,view_quality,view_angle_deg,focus_dist,focus_dir,cycles_till_next_see,"
    "see_n_players,see_n_unmatched,"
    // raw sense_body
    "sb_view_quality,sb_view_width,sb_stamina,sb_effort,sb_stamina_capacity,sb_speed_mag,sb_speed_dir,sb_neck_dir,"
    "sb_kick_count,sb_dash_count,sb_turn_count,sb_say_count,sb_turn_neck_count,sb_catch_count,sb_move_count,"
    "sb_change_view_count,sb_change_focus_count,sb_arm_movable,sb_arm_expires,"
    "sb_pointto_dist,sb_pointto_dir,sb_pointto_count,sb_attentionto_side,sb_attentionto_unum,sb_attentionto_count,"
    "sb_tackle_expires,sb_tackle_count,sb_coll_ball,sb_coll_player,sb_coll_post,sb_charged_expires,sb_card,"
    "sb_focus_dist,sb_focus_dir,"
    // librcsc estimate of the agent itself
    "est_pos_x,est_pos_y,est_pos_err_x,est_pos_err_y,est_pos_count,est_vel_x,est_vel_y,est_vel_count,"
    "est_body,est_face,est_face_err,est_face_count,est_neck,est_stamina,est_effort,est_recovery,"
    "est_stamina_capacity,est_kickable,"
    // raw self localization output (before blending)
    "rawfix_pos_x,rawfix_pos_y,rawfix_pos_err_x,rawfix_pos_err_y,rawfix_face,rawfix_face_err,"
    // ground truth of the agent itself
    "tru_pos_x,tru_pos_y,tru_vel_x,tru_vel_y,tru_body,tru_neck,tru_stamina,tru_effort,tru_recovery,"
    "tru_stamina_capacity,tru_focus_dist,tru_focus_dir,tru_pointto_dist,tru_pointto_dir,"
    "tru_kicked,tru_tackle,tru_charged,tru_card,tru_player_type,tru_goalie,"
    // noise = observed - truth (empty if one side is missing)
    "n_est_pos_dx,n_est_pos_dy,n_est_pos_d,n_est_vel_dx,n_est_vel_dy,n_est_vel_d,"
    "n_est_body,n_est_neck,n_est_face,n_est_stamina,"
    "n_rawfix_pos_dx,n_rawfix_pos_dy,n_rawfix_pos_d,n_rawfix_face,"
    "n_sb_speed_mag,n_sb_speed_dir,n_sb_neck,n_sb_stamina,"
    // commands
    "cmd_raw,body_cmd,dash_power,dash_dir,turn_moment,kick_power,kick_dir,tackle_dir,tackle_foul,catch_dir,"
    "move_x,move_y,turn_neck_moment,change_view,change_focus_dist,change_focus_dir,say_msg,"
    "pointto_dist,pointto_dir,pointto_off,attentionto_side,attentionto_unum,attentionto_off,other_sent,"
    // predictions of the effector
    "next_self_pos_x,next_self_pos_y,next_self_body,next_ball_pos_x,next_ball_pos_y,"
    "next_ball_vel_x,next_ball_vel_y,pred_next_view_width,pred_next_see_cycles,"
    // behavior label
    "action_label";

const char * const COLS_OBJECTS =
    DN_KEYS ","
    "object_id,obj_side,obj_unum,is_teammate,is_goalie,player_type,"
    "truth_valid,tru_on_field,tru_pos_x,tru_pos_y,tru_vel_x,tru_vel_y,tru_body,tru_neck,tru_stamina,tru_effort,"
    "tru_recovery,tru_stamina_capacity,tru_kicked,tru_tackle,tru_charged,tru_card,"
    "true_dist,true_dir,true_focus_dist,true_dch,true_dirch,in_cone,within_visible_dist,"
    // noise = observed - truth (empty if one side is missing)
    "n_dist,n_dist_rel,n_dir,n_dist_chg,n_dir_chg,n_body_rel,n_head_rel,"
    "n_ls_rpos_dx,n_ls_rpos_dy,n_ls_rpos_d,n_ls_pos_dx,n_ls_pos_dy,n_ls_pos_d,"
    "n_ls_vel_dx,n_ls_vel_dy,n_ls_vel_d,n_ls_rvel_dx,n_ls_rvel_dy,n_ls_rvel_d,"
    "n_trk_pos_dx,n_trk_pos_dy,n_trk_pos_d,n_trk_vel_dx,n_trk_vel_dy,n_trk_vel_d,"
    "n_trk_body,n_trk_face,n_trk_heard_pos_d,n_trk_heard_vel_d,"
    "seen,form,team_visible,unum_visible,dist,dir,dist_chg,dir_chg,body_rel,head_rel,pointto_rel,"
    "kick_flag,tackle_flag,match_method,match_err_deg,match_ambiguous,raw_dup_dropped,"
    "ls_valid,ls_rel_valid,ls_pos_valid,ls_vel_valid,ls_rvel_valid,"
    "ls_pos_x,ls_pos_y,ls_pos_err_x,ls_pos_err_y,ls_vel_x,ls_vel_y,ls_vel_err_x,ls_vel_err_y,ls_vel_count,"
    "ls_rpos_x,ls_rpos_y,ls_rpos_err_x,ls_rpos_err_y,ls_rvel_x,ls_rvel_y,ls_rvel_err_x,ls_rvel_err_y,ls_dist_error,"
    "track_found,track_match,track_match_dist,trk_unum_known,trk_pos_x,trk_pos_y,trk_vel_x,trk_vel_y,"
    "trk_body,trk_face,trk_pos_err_x,trk_pos_err_y,trk_vel_err_x,trk_vel_err_y,"
    "trk_pos_count,trk_seen_pos_count,trk_vel_count,trk_seen_vel_count,trk_body_count,trk_face_count,"
    "trk_is_ghost,trk_ghost_count,trk_lost_count,"
    "trk_heard_pos_x,trk_heard_pos_y,trk_heard_pos_count,trk_heard_vel_x,trk_heard_vel_y,trk_heard_vel_count,"
    "trk_dist_from_self,trk_angle_from_self";

const char * const COLS_LANDMARKS =
    DN_KEYS ","
    "landmark_name,landmark_id,kind,dist,dir,dist_chg,dir_chg,land_x,land_y,"
    "true_dist,true_dir,true_focus_dist,n_dist,n_dist_rel,n_dir";

const char * const COLS_HEAR =
    DN_KEYS ","
    "hear_cycle,sender_kind,sender_side,sender_unum,hear_dir,has_content,raw_msg,decoded,"
    "sender_true_x,sender_true_y,sender_true_dist,sender_true_dir_rel,"
    // noise of the decoded payload = decoded - truth of this cycle (empty if not available)
    "n_ball_pos_dx,n_ball_pos_dy,n_ball_pos_d,n_ball_vel_dx,n_ball_vel_dy,n_ball_vel_d,"
    "n_goalie_pos_dx,n_goalie_pos_dy,n_goalie_pos_d,"
    "n_player_n,n_player_pos_d_mean,n_player_pos_d_max";

const char * const COLS_EXTRA_TRACKS =
    DN_KEYS ","
    "track_list,trk_side,trk_unum,pos_x,pos_y,vel_x,vel_y,body,face,"
    "pos_count,vel_count,body_count,face_count,is_ghost,dist_from_self,angle_from_self,reason";

const char * const COLS_RAW_STREAM =
    DN_KEYS ","
    "local_ms,dir,msg";

//! marker names. the order is the same as rcsc::MarkerID
const char * const MARKER_NAMES[] = {
    "Goal_L", "Goal_R",
    "Flag_C", "Flag_CT", "Flag_CB", "Flag_LT", "Flag_LB", "Flag_RT", "Flag_RB",
    "Flag_PLT", "Flag_PLC", "Flag_PLB", "Flag_PRT", "Flag_PRC", "Flag_PRB",
    "Flag_GLT", "Flag_GLB", "Flag_GRT", "Flag_GRB",
    "Flag_TL50", "Flag_TL40", "Flag_TL30", "Flag_TL20", "Flag_TL10",
    "Flag_T0",
    "Flag_TR10", "Flag_TR20", "Flag_TR30", "Flag_TR40", "Flag_TR50",
    "Flag_BL50", "Flag_BL40", "Flag_BL30", "Flag_BL20", "Flag_BL10",
    "Flag_B0",
    "Flag_BR10", "Flag_BR20", "Flag_BR30", "Flag_BR40", "Flag_BR50",
    "Flag_LT30", "Flag_LT20", "Flag_LT10",
    "Flag_L0",
    "Flag_LB10", "Flag_LB20", "Flag_LB30",
    "Flag_RT30", "Flag_RT20", "Flag_RT10",
    "Flag_R0",
    "Flag_RB10", "Flag_RB20", "Flag_RB30"
};
const int MARKER_NAME_SIZE = static_cast< int >( sizeof( MARKER_NAMES ) / sizeof( MARKER_NAMES[0] ) );

// ---------------------------------------------------------------------------
// small utilities
// ---------------------------------------------------------------------------

std::string
csv_escape( const std::string & s )
{
    if ( s.find_first_of( ",\"\r\n" ) == std::string::npos )
    {
        return s;
    }

    std::string ret = "\"";
    for ( char c : s )
    {
        if ( c == '"' ) ret += "\"\"";
        else if ( c == '\n' || c == '\r' ) ret += ' '; // other fields are flattened. the raw stream msg is pre-escaped by raw_escape()
        else ret += c;
    }
    ret += '"';
    return ret;
}

//! reversible escape of the raw stream msg field: '\\' -> "\\\\", LF -> "\\n", CR -> "\\r", NUL is dropped
std::string
raw_escape( const std::string & s )
{
    std::string ret;
    ret.reserve( s.size() );
    for ( char c : s )
    {
        if ( c == '\\' ) ret += "\\\\";
        else if ( c == '\n' ) ret += "\\n";
        else if ( c == '\r' ) ret += "\\r";
        else if ( c == '\0' ) continue;
        else ret += c;
    }
    return ret;
}

std::string
json_escape( const std::string & s )
{
    std::string ret;
    for ( char c : s )
    {
        switch ( c ) {
        case '"': ret += "\\\""; break;
        case '\\': ret += "\\\\"; break;
        case '\n': ret += "\\n"; break;
        case '\r': ret += "\\r"; break;
        case '\t': ret += "\\t"; break;
        default:
            if ( static_cast< unsigned char >( c ) < 0x20 )
            {
                char buf[8];
                std::snprintf( buf, sizeof( buf ), "\\u%04x", static_cast< unsigned int >( c ) );
                ret += buf;
            }
            else
            {
                ret += c;
            }
            break;
        }
    }
    return ret;
}

std::string
fmt_double( const double v,
            const int prec )
{
    char buf[64];
    std::snprintf( buf, sizeof( buf ), "%.*f", prec, v );
    std::string s = buf;
    // avoid "-0.0000"
    if ( s.size() > 1 && s[0] == '-' )
    {
        bool zero = true;
        for ( std::size_t i = 1; i < s.size(); ++i )
        {
            if ( s[i] != '0' && s[i] != '.' ) { zero = false; break; }
        }
        if ( zero ) s.erase( 0, 1 );
    }
    return s;
}

std::string
join( const std::vector< std::string > & v,
      const char sep )
{
    std::string ret;
    for ( std::size_t i = 0; i < v.size(); ++i )
    {
        if ( i > 0 ) ret += sep;
        ret += v[i];
    }
    return ret;
}

std::vector< std::string >
split_ws( const std::string & s )
{
    std::vector< std::string > ret;
    std::istringstream is( s );
    std::string t;
    while ( is >> t ) ret.push_back( t );
    return ret;
}

// side_char() is rcsc::side_char( SideID ) of rcsc/types.h ('l', 'r' or 'n')

SideID
opposite( const SideID s )
{
    return ( s == LEFT ? RIGHT : s == RIGHT ? LEFT : NEUTRAL );
}

//! normalize an angle in degrees to [-180, 180)
double
norm180( double x )
{
    x = std::fmod( x + 180.0, 360.0 );
    if ( x < 0.0 ) x += 360.0;
    return x - 180.0;
}

//! global object id. 0 = ball, 1-11 = left unum, 12-22 = right unum
int
object_id( const SideID side,
           const int unum )
{
    if ( unum < 1 || 11 < unum ) return -1;
    return ( side == LEFT ? unum : side == RIGHT ? 11 + unum : -1 );
}

std::string
view_width_name( const ViewWidth::Type t )
{
    return ( t == ViewWidth::NARROW ? "narrow"
             : t == ViewWidth::NORMAL ? "normal"
             : t == ViewWidth::WIDE ? "wide"
             : "unknown" );
}

std::string
view_quality_name( const ViewQuality::Type t )
{
    return ( t == ViewQuality::HIGH ? "high" : "low" );
}

std::string
card_name( const Card c )
{
    return ( c == YELLOW ? "yellow" : c == RED ? "red" : "none" );
}

//! make a single directory. return true if it exists after the call.
bool
make_one_dir( const std::string & path )
{
    if ( path.empty() ) return true;
#ifdef _WIN32
    const int r = ::_mkdir( path.c_str() );
#else
    const int r = ::mkdir( path.c_str(), 0755 );
#endif
    if ( r == 0 ) return true;

    struct stat st;
    return ( ::stat( path.c_str(), &st ) == 0 && ( st.st_mode & S_IFDIR ) != 0 );
}

//! mkdir -p
bool
make_dirs( const std::string & dir )
{
    std::string path = dir;
    std::replace( path.begin(), path.end(), '\\', '/' );

    bool ok = true;
    for ( std::size_t i = 1; i <= path.size(); ++i )
    {
        if ( i == path.size() || path[i] == '/' )
        {
            const std::string sub = path.substr( 0, i );
            // skip a drive letter or an empty element
            if ( sub.empty() || sub.back() == ':' || sub.back() == '/' ) continue;
            ok = make_one_dir( sub );
        }
    }
    return ok;
}

std::string
sanitize_name( const std::string & s )
{
    std::string ret;
    for ( char c : s )
    {
        const bool good = ( ( 'a' <= c && c <= 'z' )
                            || ( 'A' <= c && c <= 'Z' )
                            || ( '0' <= c && c <= '9' )
                            || c == '_' || c == '-' || c == '.' );
        ret += ( good ? c : '_' );
    }
    return ret;
}

// ---------------------------------------------------------------------------
// schema, row, table
// ---------------------------------------------------------------------------

struct Schema {
    std::string header_;
    std::vector< std::string > cols_;
    std::unordered_map< std::string, std::size_t > index_;

    explicit
    Schema( const char * header )
        : header_( header )
      {
          std::string t;
          for ( char c : header_ )
          {
              if ( c == ',' ) { cols_.push_back( t ); t.clear(); }
              else t += c;
          }
          cols_.push_back( t );
          for ( std::size_t i = 0; i < cols_.size(); ++i )
          {
              index_[cols_[i]] = i;
          }
      }
};

class Row {
private:
    const Schema * M_schema;
    std::vector< std::string > M_values;

    std::size_t indexOf( const char * name )
      {
          const auto it = M_schema->index_.find( name );
          if ( it == M_schema->index_.end() )
          {
              static bool s_warned = false;
              if ( ! s_warned )
              {
                  s_warned = true;
                  std::cerr << "(DenoiseLogger) unknown column name [" << name << "]" << std::endl;
              }
              return M_values.size();
          }
          return it->second;
      }

public:
    explicit
    Row( const Schema & schema )
        : M_schema( &schema ),
          M_values( schema.cols_.size() )
      { }

    Row & s( const char * name, const std::string & v )
      {
          const std::size_t i = indexOf( name );
          if ( i < M_values.size() ) M_values[i] = csv_escape( v );
          return *this;
      }

    Row & d( const char * name, const double v, const int prec = PREC )
      {
          if ( std::isfinite( v ) )
          {
              const std::size_t i = indexOf( name );
              if ( i < M_values.size() ) M_values[i] = fmt_double( v, prec );
          }
          return *this;
      }

    Row & i( const char * name, const long long v )
      {
          const std::size_t idx = indexOf( name );
          if ( idx < M_values.size() ) M_values[idx] = std::to_string( v );
          return *this;
      }

    Row & b( const char * name, const bool v )
      {
          const std::size_t idx = indexOf( name );
          if ( idx < M_values.size() ) M_values[idx] = ( v ? "1" : "0" );
          return *this;
      }

    //! x and y of a vector. nothing is written if the vector is invalid.
    Row & xy( const char * name_x, const char * name_y, const Vector2D & v, const int prec = PREC )
      {
          if ( v.isValid() )
          {
              d( name_x, v.x, prec );
              d( name_y, v.y, prec );
          }
          return *this;
      }

    //! noise of a scalar (observed - truth). nothing is written if one of them is not finite.
    Row & nd( const char * name, const double obs, const double tru )
      {
          if ( std::isfinite( obs ) && std::isfinite( tru ) )
          {
              d( name, obs - tru, PREC_TRUE );
          }
          return *this;
      }

    //! noise of an angle in degrees, normalized to [-180, 180)
    Row & nang( const char * name, const double obs, const double tru )
      {
          if ( std::isfinite( obs ) && std::isfinite( tru ) )
          {
              d( name, norm180( obs - tru ), PREC_TRUE );
          }
          return *this;
      }

    //! Euclidean norm of the noise of a vector only
    Row & dnorm( const char * name_d, const Vector2D & obs, const Vector2D & tru )
      {
          if ( obs.isValid() && tru.isValid()
               && std::isfinite( obs.x ) && std::isfinite( obs.y )
               && std::isfinite( tru.x ) && std::isfinite( tru.y ) )
          {
              d( name_d, std::sqrt( ( obs.x - tru.x ) * ( obs.x - tru.x )
                                    + ( obs.y - tru.y ) * ( obs.y - tru.y ) ), PREC_TRUE );
          }
          return *this;
      }

    //! noise of a vector: dx, dy and the Euclidean norm. nothing is written if one of them is invalid.
    Row & dxy( const char * name_dx, const char * name_dy, const char * name_d,
               const Vector2D & obs, const Vector2D & tru )
      {
          if ( obs.isValid() && tru.isValid()
               && std::isfinite( obs.x ) && std::isfinite( obs.y )
               && std::isfinite( tru.x ) && std::isfinite( tru.y ) )
          {
              const double dx = obs.x - tru.x;
              const double dy = obs.y - tru.y;
              d( name_dx, dx, PREC_TRUE );
              d( name_dy, dy, PREC_TRUE );
              d( name_d, std::sqrt( dx * dx + dy * dy ), PREC_TRUE );
          }
          return *this;
      }

    std::string line() const
      {
          return join( M_values, ',' );
      }
};

class Table {
private:
    const Schema * M_schema;
    std::string M_name;
    std::unique_ptr< std::ostream > M_os;
    std::string M_path;
#ifdef HAVE_LIBZ
    gzofstream * M_file = nullptr; //!< alias of M_os (concrete type, to close it explicitly)
#else
    std::ofstream * M_file = nullptr; //!< alias of M_os
#endif

public:
    Table( const Schema & schema, const char * name )
        : M_schema( &schema ),
          M_name( name )
      { }

    const std::string & name() const { return M_name; }
    const Schema & schema() const { return *M_schema; }
    const std::string & path() const { return M_path; }
    bool isOpen() const { return M_os.get() != nullptr; }

    bool open( const std::string & dir )
      {
#ifdef HAVE_LIBZ
          M_path = dir + "/" + M_name + ".csv.gz";
          gzofstream * gz = new gzofstream( M_path.c_str() );
          if ( ! gz->is_open() )
          {
              delete gz;
              std::cerr << "(DenoiseLogger) failed to open " << M_path << std::endl;
              return false;
          }
          M_os.reset( gz );
          M_file = gz;
#else
          M_path = dir + "/" + M_name + ".csv";
          std::ofstream * fout = new std::ofstream( M_path.c_str() );
          if ( ! fout->is_open() )
          {
              delete fout;
              std::cerr << "(DenoiseLogger) failed to open " << M_path << std::endl;
              return false;
          }
          M_os.reset( fout );
          M_file = fout;
#endif
          *M_os << M_schema->header_ << '\n';
          if ( ! M_os->good() )
          {
              std::cerr << "(DenoiseLogger) failed to write " << M_path << std::endl;
              M_file = nullptr;
              M_os.reset();
              return false;
          }
          return true;
      }

    void write( const Row & row )
      {
          if ( M_os ) *M_os << row.line() << '\n';
      }

    void writeLine( const std::string & line )
      {
          if ( M_os ) *M_os << line << '\n';
      }

    //! true if the stream is open and in a failed state
    bool bad() const
      {
          return ( M_os && ! M_os->good() );
      }

    void flush()
      {
          if ( M_os ) M_os->flush();
      }

    //! flush and explicitly close the concrete stream (gzclose / fclose), then check the state.
    //! \return false if a flush or the close failed
    bool close()
      {
          bool ok = true;
          if ( M_os )
          {
              M_os->flush();
              ok = M_os->good();
              if ( M_file )
              {
                  M_file->close(); // sets failbit if the close failed
                  ok = ok && M_file->good();
              }
              M_file = nullptr;
              M_os.reset();
          }
          return ok;
      }
};

const Schema SCHEMA_AGENT_CYCLE( COLS_AGENT_CYCLE );
const Schema SCHEMA_OBJECTS( COLS_OBJECTS );
const Schema SCHEMA_LANDMARKS( COLS_LANDMARKS );
const Schema SCHEMA_HEAR( COLS_HEAR );
const Schema SCHEMA_EXTRA_TRACKS( COLS_EXTRA_TRACKS );
const Schema SCHEMA_RAW_STREAM( COLS_RAW_STREAM );

// ---------------------------------------------------------------------------
// command parser
// ---------------------------------------------------------------------------

struct ParsedCommand {
    std::string name_;
    std::vector< std::string > args_;
    std::string quoted_; //!< text in the double quotation
};

//! split a command batch "(dash 100)(turn_neck 30)(say "abc")" into commands
std::vector< ParsedCommand >
parse_commands( const std::string & batch )
{
    std::vector< ParsedCommand > ret;

    std::size_t i = 0;
    while ( i < batch.size() )
    {
        if ( batch[i] != '(' ) { ++i; continue; }

        std::size_t j = i + 1;
        bool in_quote = false;
        int depth = 1;
        for ( ; j < batch.size(); ++j )
        {
            const char c = batch[j];
            if ( c == '"' ) in_quote = ! in_quote;
            else if ( ! in_quote && c == '(' ) ++depth;
            else if ( ! in_quote && c == ')' )
            {
                if ( --depth == 0 ) break;
            }
        }

        std::string body = batch.substr( i + 1, j - i - 1 );
        i = j + 1;

        ParsedCommand cmd;
        const std::size_t q1 = body.find( '"' );
        if ( q1 != std::string::npos )
        {
            const std::size_t q2 = body.rfind( '"' );
            if ( q2 > q1 ) cmd.quoted_ = body.substr( q1 + 1, q2 - q1 - 1 );
            body = body.substr( 0, q1 );
        }

        std::vector< std::string > tok = split_ws( body );
        if ( tok.empty() ) continue;
        cmd.name_ = tok.front();
        cmd.args_.assign( tok.begin() + 1, tok.end() );
        ret.push_back( cmd );
    }

    return ret;
}

} // end of anonymous namespace


/*-------------------------------------------------------------------*/
/*!
  implementation
*/
struct DenoiseLogger::Impl {

    //! ground truth of one object (global frame)
    struct TruthObj {
        bool has_ = false;
        SideID side_ = NEUTRAL;
        int unum_ = 0;
        bool goalie_ = false;
        int type_ = -1;
        Vector2D pos_;
        Vector2D vel_;
        double body_ = 0.0;
        double neck_ = 0.0;
        double stamina_ = 0.0;
        double effort_ = 0.0;
        double recovery_ = 0.0;
        double capacity_ = 0.0;
        double focus_dist_ = 0.0;
        double focus_dir_ = 0.0;
        double pointto_dist_ = -1.0;
        double pointto_dir_ = 0.0;
        bool kicked_ = false;
        bool tackle_ = false;
        bool charged_ = false;
        Card card_ = NO_CARD;
    };

    //! matched raw see player
    struct SeenRec {
        bool has_ = false;
        VisualSensor::PlayerT p_;
        int kind_ = 0; //!< 0: unum known, 1: team only, 2: anonymous
        std::string method_;
        double err_ = 0.0;
        bool amb_ = false;
        int dup_ = 0;
        bool rec_has_ = false; //!< true if the localization record of this entry exists (this cycle)
        WorldModel::SeePlayerRecord rec_;
    };

    //! matched world model track
    struct TrackAsg {
        const PlayerObject * obj_ = nullptr;
        const char * method_ = "";
        const char * list_ = "";
        double dist_ = std::numeric_limits< double >::quiet_NaN();
    };

    //! track that has no truth slot
    struct ExtraTrack {
        const PlayerObject * obj_ = nullptr;
        const char * list_ = "";
        const char * reason_ = "";
    };

    //! buffered hear message
    struct PendingHear {
        int hear_cycle_ = 0;
        GameTime time_;
        std::string kind_;
        SideID sender_side_ = NEUTRAL;
        int sender_unum_ = -1;
        bool has_dir_ = false;
        double dir_ = 0.0;
        bool has_content_ = false;
        std::string msg_;
    };

    //! raw stream row buffered until the output directory is known
    struct PendingRaw {
        std::string dir_;
        std::string msg_;
        long long ms_ = 0;
        int cycle_ = 0;
        int stopped_ = 0;
    };

    const PlayerAgent & agent_;

    bool opened_ = false;
    bool open_failed_ = false;
    bool closed_ = false;
    bool meta_written_ = false;
    bool failed_ = false; //!< true after the first write failure. all logging is stopped.
    std::string error_msg_;

    std::string root_dir_;
    std::string dir_;
    std::string game_id_;
    std::string created_;
    SideID side_ = NEUTRAL;
    int unum_ = 0;
    int self_id_ = -1;

    Table t_cycle_;
    Table t_obj_;
    Table t_land_;
    Table t_hear_;
    Table t_extra_;
    Table t_raw_;

    ObjectTable object_table_;

    // cycle state
    bool in_cycle_ = false;
    GameTime cur_;
    GameTime last_time_;
    std::string trigger_;
    std::vector< std::string > labels_;
    std::vector< std::string > other_sent_;
    std::unique_ptr< Row > row_cycle_;
    int cycles_since_flush_ = 0;
    GameTime last_row_time_; //!< time of the last agent_cycle row (real or placeholder)

    // predictions snapshot taken by preSend() (before ActionEffector::makeCommand() deletes the commands)
    bool pre_valid_ = false;
    Vector2D pre_self_pos_;
    double pre_self_body_ = 0.0;
    Vector2D pre_ball_pos_;
    Vector2D pre_ball_vel_;
    ViewWidth::Type pre_view_width_ = ViewWidth::NORMAL;
    int pre_see_cycles_ = 0;

    long long last_sb_recv_ms_ = -1;
    int last_sb_cycle_ = -1;
    std::map< std::pair< int, int >, int > see_counts_; //!< key = (cycle, stopped) of the see message
    std::vector< PendingHear > hear_pending_;
    std::vector< PendingRaw > raw_pending_;

    // per cycle truth and matching results
    bool truth_valid_ = false;
    std::array< TruthObj, 23 > truth_;
    bool seen_ball_ = false;
    VisualSensor::BallT seen_ball_data_;
    std::array< SeenRec, 23 > seen_;
    int n_players_ = 0;
    int n_unmatched_ = 0;
    std::array< TrackAsg, 23 > trk_;
    std::vector< ExtraTrack > extra_;

    explicit
    Impl( const PlayerAgent & agent )
        : agent_( agent ),
          t_cycle_( SCHEMA_AGENT_CYCLE, "agent_cycle" ),
          t_obj_( SCHEMA_OBJECTS, "objects" ),
          t_land_( SCHEMA_LANDMARKS, "landmarks" ),
          t_hear_( SCHEMA_HEAR, "hear" ),
          t_extra_( SCHEMA_EXTRA_TRACKS, "extra_tracks" ),
          t_raw_( SCHEMA_RAW_STREAM, "raw_stream" ),
          cur_( -1, 0 ),
          last_time_( -1, 0 ),
          last_row_time_( -1, 0 )
      { }

    // ---- coordinate helpers (team normalized frame -> global frame)

    double sgn() const
      {
          return ( agent_.world().ourSide() == RIGHT ? -1.0 : 1.0 );
      }

    Vector2D g( const Vector2D & v ) const
      {
          if ( ! v.isValid() ) return v; // keep the invalid marker
          const double s = sgn();
          return Vector2D( v.x * s, v.y * s );
      }

    double ga( const double deg ) const
      {
          return AngleDeg::normalize_angle( agent_.world().ourSide() == RIGHT ? deg + 180.0 : deg );
      }

    void keys( Row & r, const GameTime & t ) const
      {
          r.s( "game_id", game_id_ )
              .s( "side", std::string( 1, side_char( side_ ) ) )
              .i( "unum", unum_ )
              .i( "cycle", t.cycle() )
              .i( "stopped", t.stopped() );
      }

    // ---- open / close

    bool ensureOpen();
    bool writeMeta( const bool final_write );
    void fail( const std::string & msg );
    void checkFailure();
    void flushAll();
    void snapshotPredictions();

    // ---- per cycle

    void buildTruth( const GameTime & t );
    bool relTo( const Vector2D & target, double * dist, double * dir, double * focus_dist ) const;
    void matchSeenPlayers( const GameTime & t );
    void matchTracks();
    void buildCyclePrefix();
    void writeObjects();
    void writeLandmarks();
    void writeExtraTracks();
    void flushHear( const GameTime & t, const bool all );
    std::string decodeHear( const PendingHear & h ) const;
    void hearNoise( Row & r, const PendingHear & h ) const;
    void addHear( const char * msg, const GameTime & current );
    void addRaw( const char * dir, const std::string & msg, const long long ms, const GameTime & t );
    void beginCycle( const GameTime & t, const std::string & trigger );
    void endCycle( const std::string & batch );
    bool closeAll();
};

namespace {

int
header_cycle( const char * msg,
              const int fallback )
{
    if ( ! std::strncmp( msg, "(see ", 5 )
         || ! std::strncmp( msg, "(hear ", 6 )
         || ! std::strncmp( msg, "(sense_body ", 12 )
         || ! std::strncmp( msg, "(fullstate ", 11 ) )
    {
        const char * p = std::strchr( msg, ' ' );
        int t = 0;
        if ( p && std::sscanf( p + 1, "%d", &t ) == 1 )
        {
            return t;
        }
    }
    return fallback;
}

std::string
trim_string( const std::string & s )
{
    const std::size_t b = s.find_first_not_of( " \t\r\n" );
    if ( b == std::string::npos ) return std::string();
    const std::size_t e = s.find_last_not_of( " \t\r\n" );
    return s.substr( b, e - b + 1 );
}

//! parse "(name value)(name value)..." (with an optional (server_param ...) wrapper)
std::vector< std::pair< std::string, std::string > >
parse_param_string( const std::string & src )
{
    std::vector< std::pair< std::string, std::string > > ret;

    std::string s = trim_string( src );
    if ( s.size() > 2 && s[0] == '(' )
    {
        const std::size_t e = s.find_first_of( " \t\r\n(", 1 );
        const std::string name = s.substr( 1, e == std::string::npos ? std::string::npos : e - 1 );
        if ( name == "server_param" || name == "player_param" )
        {
            s = s.substr( e == std::string::npos ? s.size() : e );
            s = trim_string( s );
            if ( ! s.empty() && s.back() == ')' ) s.pop_back();
        }
    }

    std::size_t i = 0;
    while ( i < s.size() )
    {
        if ( s[i] != '(' ) { ++i; continue; }

        std::size_t j = i + 1;
        while ( j < s.size() && s[j] != ' ' && s[j] != ')' ) ++j;
        const std::string name = s.substr( i + 1, j - i - 1 );

        std::size_t k = j;
        bool in_quote = false;
        int depth = 1;
        for ( ; k < s.size(); ++k )
        {
            const char c = s[k];
            if ( c == '"' ) in_quote = ! in_quote;
            else if ( ! in_quote && c == '(' ) ++depth;
            else if ( ! in_quote && c == ')' )
            {
                if ( --depth == 0 ) break;
            }
        }

        std::string value = trim_string( s.substr( j, k > j ? k - j : 0 ) );
        if ( value.size() >= 2 && value.front() == '"' && value.back() == '"' )
        {
            value = value.substr( 1, value.size() - 2 );
        }
        if ( ! name.empty() ) ret.push_back( std::make_pair( name, value ) );
        i = k + 1;
    }

    return ret;
}

void
append_params_json( std::ostream & os,
                    const std::string & key,
                    const std::string & src )
{
    os << "  \"" << key << "\": {";
    const auto params = parse_param_string( src );
    for ( std::size_t i = 0; i < params.size(); ++i )
    {
        os << ( i == 0 ? "\n" : ",\n" )
           << "    \"" << json_escape( params[i].first ) << "\": \"" << json_escape( params[i].second ) << '"';
    }
    os << "\n  }";
}

std::string
current_timestamp( const char * format )
{
    const std::time_t t = std::time( nullptr );
    char buf[64];
    std::strftime( buf, sizeof( buf ), format, std::localtime( &t ) );
    return buf;
}

std::string
vec_text( const Vector2D & v )
{
    if ( ! v.isValid() ) return "?"; // invalid vector (should not happen for decoded messages)
    return fmt_double( v.x, PREC ) + " " + fmt_double( v.y, PREC );
}

} // end of anonymous namespace

/*-------------------------------------------------------------------*/
bool
DenoiseLogger::Impl::ensureOpen()
{
    if ( opened_ ) return true;
    if ( open_failed_ || closed_ ) return false;

    const WorldModel & wm = agent_.world();
    if ( wm.ourSide() == NEUTRAL || wm.self().unum() < 1 )
    {
        return false;
    }

    side_ = wm.ourSide();
    unum_ = wm.self().unum();
    self_id_ = object_id( side_, unum_ );

    root_dir_ = agent_.config().denoiseLogDir();
    std::replace( root_dir_.begin(), root_dir_.end(), '\\', '/' );
    while ( root_dir_.size() > 1 && root_dir_.back() == '/' ) root_dir_.pop_back();

    game_id_ = agent_.config().denoiseGameId();
    if ( game_id_.empty() )
    {
        game_id_ = current_timestamp( "%Y%m%d_%H%M%S" ) + "_" + wm.ourTeamName();
        if ( ! wm.theirTeamName().empty() ) game_id_ += "_vs_" + wm.theirTeamName();
    }
    game_id_ = sanitize_name( game_id_ );

    created_ = current_timestamp( "%Y-%m-%dT%H:%M:%S" );
    dir_ = root_dir_ + "/" + game_id_ + "/" + std::string( 1, side_char( side_ ) ) + "_" + std::to_string( unum_ );

    if ( ! make_dirs( dir_ ) )
    {
        std::cerr << "(DenoiseLogger) failed to create the directory " << dir_ << std::endl;
        open_failed_ = true;
        return false;
    }

    bool ok = true;
    ok = t_cycle_.open( dir_ ) && ok;
    ok = t_obj_.open( dir_ ) && ok;
    ok = t_hear_.open( dir_ ) && ok;
    ok = t_extra_.open( dir_ ) && ok;
    if ( agent_.config().denoiseLogLandmarks() ) ok = t_land_.open( dir_ ) && ok;
    if ( agent_.config().denoiseLogRaw() ) ok = t_raw_.open( dir_ ) && ok;

    if ( ! ok )
    {
        open_failed_ = true;
        closeAll();
        return false;
    }

    opened_ = true;

    // write the raw messages received before the id of this agent was known
    for ( const PendingRaw & p : raw_pending_ )
    {
        Row r( SCHEMA_RAW_STREAM );
        keys( r, GameTime( p.cycle_, p.stopped_ ) );
        r.i( "local_ms", p.ms_ ).s( "dir", p.dir_ ).s( "msg", raw_escape( p.msg_ ) );
        t_raw_.write( r );
    }
    raw_pending_.clear();

    return true;
}

/*-------------------------------------------------------------------*/
bool
DenoiseLogger::Impl::closeAll()
{
    bool ok = true;
    ok = t_cycle_.close() && ok;
    ok = t_obj_.close() && ok;
    ok = t_land_.close() && ok;
    ok = t_hear_.close() && ok;
    ok = t_extra_.close() && ok;
    ok = t_raw_.close() && ok;
    return ok;
}

/*-------------------------------------------------------------------*/
/*!
  first write failure: report once, stop logging, record the error in meta.json
 */
void
DenoiseLogger::Impl::fail( const std::string & msg )
{
    if ( failed_ ) return;

    failed_ = true;
    error_msg_ = msg;
    std::cerr << "(DenoiseLogger) " << msg << ". logging is stopped. dir=" << dir_ << std::endl;

    closeAll();
    writeMeta( false ); // best effort. records status = error
}

/*-------------------------------------------------------------------*/
void
DenoiseLogger::Impl::checkFailure()
{
    if ( failed_ ) return;

    const Table * tables[] = { &t_cycle_, &t_obj_, &t_land_, &t_hear_, &t_extra_, &t_raw_ };
    for ( const Table * t : tables )
    {
        if ( t->bad() )
        {
            fail( "write failure: " + t->path() );
            return;
        }
    }
}

/*-------------------------------------------------------------------*/
void
DenoiseLogger::Impl::flushAll()
{
    t_cycle_.flush();
    t_obj_.flush();
    t_land_.flush();
    t_hear_.flush();
    t_extra_.flush();
    t_raw_.flush();
    checkFailure();
}

/*-------------------------------------------------------------------*/
/*!
  snapshot of the effector predictions. the queued command objects are deleted by
  ActionEffector::makeCommand(), so this must be called before it.
 */
void
DenoiseLogger::Impl::snapshotPredictions()
{
    const ActionEffector & ef = agent_.effector();

    pre_self_pos_ = ef.queuedNextSelfPos();
    pre_self_body_ = ef.queuedNextSelfBody().degree();
    pre_ball_pos_ = ef.queuedNextBallPos();
    pre_ball_vel_ = ef.queuedNextBallVel();
    pre_view_width_ = ef.queuedNextViewWidth().type();
    pre_see_cycles_ = ef.queuedNextSeeCycles();
    pre_valid_ = true;
}

/*-------------------------------------------------------------------*/
bool
DenoiseLogger::Impl::writeMeta( const bool final_write )
{
    if ( ! opened_ ) return true;

    const WorldModel & wm = agent_.world();
    const PlayerConfig & cfg = agent_.config();

    std::ofstream os( ( dir_ + "/meta.json" ).c_str() );
    if ( ! os.is_open() ) return false;

    const PlayerType & self_type = wm.self().playerType();

    os << "{\n";
    os << "  \"schema_version\": " << SCHEMA_VERSION << ",\n";
    os << "  \"game_id\": \"" << json_escape( game_id_ ) << "\",\n";
    os << "  \"created\": \"" << created_ << "\",\n";
    // final = true only if the logger was closed cleanly (no write failure)
    os << "  \"final\": " << ( final_write && ! failed_ ? "true" : "false" ) << ",\n";
    os << "  \"status\": \"" << ( failed_ ? "error" : "ok" ) << "\",\n";
    if ( failed_ ) os << "  \"error\": \"" << json_escape( error_msg_ ) << "\",\n";
    os << "  \"side\": \"" << side_char( side_ ) << "\",\n";
    os << "  \"unum\": " << unum_ << ",\n";
    os << "  \"goalie\": " << ( wm.self().goalie() ? "true" : "false" ) << ",\n";
    os << "  \"team_name\": \"" << json_escape( wm.ourTeamName() ) << "\",\n";
    os << "  \"opponent_team_name\": \"" << json_escape( wm.theirTeamName() ) << "\",\n";
    os << "  \"last_cycle\": " << last_time_.cycle() << ",\n";
    os << "  \"last_stopped\": " << last_time_.stopped() << ",\n";
    os << "  \"score_left\": " << wm.gameMode().scoreLeft() << ",\n";
    os << "  \"score_right\": " << wm.gameMode().scoreRight() << ",\n";
    os << "  \"coordinate_frame\": \"global server frame (left team frame); x+ is toward the right goal; "
       << "right side agent values are un-reversed by the logger\",\n";
    os << "  \"angle_unit\": \"degree\",\n";
    os << "  \"decimals\": " << PREC << ",\n";
    os << "  \"decimals_true_geometry\": " << PREC_TRUE << ",\n";
    os << "  \"compressed\": "
#ifdef HAVE_LIBZ
       << "true"
#else
       << "false"
#endif
       << ",\n";
    os << "  \"object_id\": \"0 = ball, 1-11 = left unum, 12-22 = right unum\",\n";

    os << "  \"config\": {\n";
    os << "    \"version\": " << cfg.version() << ",\n";
    os << "    \"gaussian_see\": " << ( cfg.gaussianSee() ? "true" : "false" ) << ",\n";
    os << "    \"use_fullstate\": " << ( cfg.useFullstate() ? "true" : "false" ) << ",\n";
    os << "    \"debug_fullstate\": " << ( cfg.debugFullstate() ? "true" : "false" ) << ",\n";
    os << "    \"synch_see\": " << ( cfg.synchSee() ? "true" : "false" ) << ",\n";
    os << "    \"denoise_log_landmarks\": " << ( cfg.denoiseLogLandmarks() ? "true" : "false" ) << ",\n";
    os << "    \"denoise_log_raw\": " << ( cfg.denoiseLogRaw() ? "true" : "false" ) << ",\n";
    os << "    \"full_view\": " << ( ServerParam::i().fullView() ? "true" : "false" ) << ",\n";
    os << "    \"see_noise_free\": " << ( ServerParam::i().seeNoiseFree() ? "true" : "false" ) << "\n";
    os << "  },\n";

    os << "  \"player_type_id\": " << self_type.id() << ",\n";

    os << "  \"tables\": {\n";
    const Table * tables[] = { &t_cycle_, &t_obj_, &t_land_, &t_hear_, &t_extra_, &t_raw_ };
    for ( std::size_t i = 0; i < 6; ++i )
    {
        os << "    \"" << tables[i]->name() << "\": [";
        const std::vector< std::string > & cols = tables[i]->schema().cols_;
        for ( std::size_t k = 0; k < cols.size(); ++k )
        {
            os << ( k == 0 ? "" : "," ) << '"' << cols[k] << '"';
        }
        os << "]" << ( i + 1 < 6 ? ",\n" : "\n" );
    }
    os << "  },\n";

    // player types
    os << "  \"player_types\": [";
    {
        std::vector< int > ids;
        for ( const auto & v : PlayerTypeSet::i().playerTypeMap() ) ids.push_back( v.first );
        std::sort( ids.begin(), ids.end() );
        bool first = true;
        for ( const int id : ids )
        {
            const PlayerType * pt = PlayerTypeSet::i().get( id );
            if ( ! pt ) continue;
            os << ( first ? "\n" : ",\n" );
            first = false;
            os << "    {\"id\": " << pt->id()
               << ", \"player_speed_max\": " << pt->playerSpeedMax()
               << ", \"stamina_inc_max\": " << pt->staminaIncMax()
               << ", \"player_decay\": " << pt->playerDecay()
               << ", \"inertia_moment\": " << pt->inertiaMoment()
               << ", \"dash_power_rate\": " << pt->dashPowerRate()
               << ", \"player_size\": " << pt->playerSize()
               << ", \"kickable_margin\": " << pt->kickableMargin()
               << ", \"kick_rand\": " << pt->kickRand()
               << ", \"extra_stamina\": " << pt->extraStamina()
               << ", \"effort_max\": " << pt->effortMax()
               << ", \"effort_min\": " << pt->effortMin()
               << ", \"kick_power_rate\": " << pt->kickPowerRate()
               << ", \"foul_detect_probability\": " << pt->foulDetectProbability()
               << ", \"catch_area_length_stretch\": " << pt->catchAreaLengthStretch()
               << ", \"unum_far_length\": " << pt->unumFarLength()
               << ", \"unum_too_far_length\": " << pt->unumTooFarLength()
               << ", \"team_far_length\": " << pt->teamFarLength()
               << ", \"team_too_far_length\": " << pt->teamTooFarLength()
               << ", \"player_max_observation_length\": " << pt->playerMaxObservationLength()
               << ", \"ball_vel_far_length\": " << pt->ballVelFarLength()
               << ", \"ball_vel_too_far_length\": " << pt->ballVelTooFarLength()
               << ", \"ball_max_observation_length\": " << pt->ballMaxObservationLength()
               << ", \"flag_chg_far_length\": " << pt->flagChgFarLength()
               << ", \"flag_chg_too_far_length\": " << pt->flagChgTooFarLength()
               << ", \"flag_max_observation_length\": " << pt->flagMaxObservationLength()
               << ", \"dist_noise_rate\": " << pt->distNoiseRate()
               << ", \"focus_dist_noise_rate\": " << pt->focusDistNoiseRate()
               << ", \"land_dist_noise_rate\": " << pt->landDistNoiseRate()
               << ", \"land_focus_dist_noise_rate\": " << pt->landFocusDistNoiseRate()
               << "}";
        }
        os << ( first ? "" : "\n  " ) << "],\n";
    }

    append_params_json( os, "server_param", ServerParam::i().toServerString() );
    os << ",\n";
    append_params_json( os, "player_param", PlayerParam::i().toServerString() );
    os << "\n}\n";

    os.flush();
    meta_written_ = true;
    return os.good();
}

/*-------------------------------------------------------------------*/
void
DenoiseLogger::Impl::buildTruth( const GameTime & t )
{
    for ( TruthObj & o : truth_ ) o = TruthObj();
    truth_valid_ = false;

    const FullstateSensor & fs = agent_.fullstateSensor();
    if ( fs.time() != t
         || ( fs.ourPlayers().empty() && fs.theirPlayers().empty() ) )
    {
        return;
    }

    // ball
    {
        TruthObj & b = truth_[0];
        b.has_ = fs.ball().pos_.isValid();
        b.pos_ = g( fs.ball().pos_ );
        b.vel_ = g( fs.ball().vel_ );
    }

    // players. fullstate values are in the team normalized frame.
    for ( int k = 0; k < 2; ++k )
    {
        const FullstateSensor::PlayerCont & cont = ( k == 0 ? fs.ourPlayers() : fs.theirPlayers() );
        const SideID side = ( k == 0 ? side_ : opposite( side_ ) );
        for ( const FullstateSensor::PlayerT & p : cont )
        {
            const int id = object_id( side, p.unum_ );
            if ( id < 1 ) continue;

            TruthObj & o = truth_[id];
            o.has_ = true;
            o.side_ = side;
            o.unum_ = p.unum_;
            o.goalie_ = p.goalie_;
            o.type_ = p.type_;
            o.pos_ = g( p.pos_ );
            o.vel_ = g( p.vel_ );
            o.body_ = ga( p.body_ );
            o.neck_ = p.neck_;
            o.stamina_ = p.stamina_;
            o.effort_ = p.effort_;
            o.recovery_ = p.recovery_;
            o.capacity_ = p.stamina_capacity_;
            o.focus_dist_ = p.focus_dist_;
            o.focus_dir_ = p.focus_dir_;
            o.pointto_dist_ = p.pointto_dist_;
            o.pointto_dir_ = p.pointto_dir_;
            o.kicked_ = p.kicked_;
            o.tackle_ = p.tackle_;
            o.charged_ = p.charged_;
            o.card_ = p.card_;
        }
    }

    truth_valid_ = ( 1 <= self_id_ && self_id_ <= 22 && truth_[self_id_].has_ );
}

/*-------------------------------------------------------------------*/
/*!
  relative geometry from the true state of this agent.
  dir is relative to the true face direction.
 */
bool
DenoiseLogger::Impl::relTo( const Vector2D & target,
                            double * dist,
                            double * dir,
                            double * focus_dist ) const
{
    if ( ! truth_valid_ || ! target.isValid() ) return false;

    const TruthObj & me = truth_[self_id_];
    const double face = me.body_ + me.neck_;

    const Vector2D rel = target - me.pos_;
    if ( dist ) *dist = rel.r();
    if ( dir ) *dir = AngleDeg::normalize_angle( rel.th().degree() - face );
    if ( focus_dist )
    {
        const Vector2D fp = me.pos_ + Vector2D::polar2vector( me.focus_dist_,
                                                              AngleDeg( face + me.focus_dir_ ) );
        *focus_dist = ( target - fp ).r();
    }
    return true;
}

/*-------------------------------------------------------------------*/
/*!
  match the raw see players to the true objects
 */
void
DenoiseLogger::Impl::matchSeenPlayers( const GameTime & t )
{
    for ( SeenRec & s : seen_ ) s = SeenRec();
    seen_ball_ = false;
    n_players_ = 0;
    n_unmatched_ = 0;

    const VisualSensor & vs = agent_.visualSensor();
    if ( vs.time() != t ) return;

    if ( ! vs.balls().empty() )
    {
        seen_ball_ = true;
        seen_ball_data_ = vs.balls().front();
    }

    // list: 0 = opponents, 1 = unknownOpponents, 2 = teammates, 3 = unknownTeammates, 4 = unknownPlayers
    // (the same numbering as WorldModel::SeePlayerRecord::list_). index_ is the position in that list.
    struct Item {
        const VisualSensor::PlayerT * p_;
        int kind_;
        SideID side_;
        int list_;
        int index_;
    };

    std::vector< Item > items;
    {
        int idx = 0;
        for ( const VisualSensor::PlayerT & p : vs.opponents() )
            items.push_back( Item{ &p, p.unum_ > 0 ? 0 : 1, opposite( side_ ), 0, idx++ } );
        idx = 0;
        for ( const VisualSensor::PlayerT & p : vs.unknownOpponents() )
            items.push_back( Item{ &p, 1, opposite( side_ ), 1, idx++ } );
        idx = 0;
        for ( const VisualSensor::PlayerT & p : vs.teammates() )
            items.push_back( Item{ &p, p.unum_ > 0 ? 0 : 1, side_, 2, idx++ } );
        idx = 0;
        for ( const VisualSensor::PlayerT & p : vs.unknownTeammates() )
            items.push_back( Item{ &p, 1, side_, 3, idx++ } );
        idx = 0;
        for ( const VisualSensor::PlayerT & p : vs.unknownPlayers() )
            items.push_back( Item{ &p, 2, NEUTRAL, 4, idx++ } );
    }

    n_players_ = static_cast< int >( items.size() );

    const WorldModel & wm = agent_.world();
    const bool rec_ok = ( wm.seePlayerRecordTime() == t );

    std::array< bool, 23 > claimed;
    claimed.fill( false );

    // slot assignment. the smaller error wins, an unum match wins over the others.
    auto assign = [this, &wm, rec_ok]( const int id, const Item & it, const char * method, const double err, const bool amb )
        {
            SeenRec & s = seen_[id];
            if ( s.has_ )
            {
                ++s.dup_;
                ++n_unmatched_;
                const bool new_better = ( s.method_ != "unum" && std::string( method ) == "unum" )
                    || ( ( s.method_ == "unum" ) == ( std::string( method ) == "unum" ) && err < s.err_ );
                if ( ! new_better ) return;
            }
            s.has_ = true;
            s.p_ = *it.p_;
            s.kind_ = it.kind_;
            s.method_ = method;
            s.err_ = err;
            s.amb_ = amb;
            s.rec_has_ = false;
            if ( rec_ok )
            {
                for ( const WorldModel::SeePlayerRecord & r : wm.seePlayerRecords() )
                {
                    if ( r.list_ == it.list_ && r.index_ == it.index_ )
                    {
                        s.rec_has_ = true;
                        s.rec_ = r;
                        break;
                    }
                }
            }
        };

    // pass 1: uniform number is known
    for ( const Item & it : items )
    {
        if ( it.kind_ != 0 ) continue;

        const int id = object_id( it.side_, it.p_->unum_ );
        // an unum match does not need the truth
        if ( id < 1 || id == self_id_ )
        {
            ++n_unmatched_;
            continue;
        }

        double d = 0.0, dir = 0.0;
        double err = std::numeric_limits< double >::quiet_NaN();
        if ( truth_valid_ && truth_[id].has_
             && relTo( truth_[id].pos_, &d, &dir, nullptr ) )
        {
            err = std::fabs( AngleDeg::normalize_angle( it.p_->dir_ - dir ) );
        }
        claimed[id] = true;
        assign( id, it, "unum", err, false );
    }

    // pass 2: nearest direction among the unclaimed objects
    for ( const Item & it : items )
    {
        if ( it.kind_ == 0 ) continue;

        if ( ! truth_valid_ )
        {
            ++n_unmatched_;
            continue;
        }

        int best = -1;
        double best_err = 1.0e10;
        double best_derr = 1.0e10;
        double second_err = 1.0e10;

        for ( int id = 1; id <= 22; ++id )
        {
            if ( id == self_id_ || claimed[id] || ! truth_[id].has_ ) continue;
            if ( it.side_ != NEUTRAL && truth_[id].side_ != it.side_ ) continue;

            double d = 0.0, dir = 0.0;
            relTo( truth_[id].pos_, &d, &dir, nullptr );
            const double err = std::fabs( AngleDeg::normalize_angle( it.p_->dir_ - dir ) );
            const double derr = std::fabs( it.p_->dist_ - d );

            if ( err < best_err
                 || ( err == best_err && derr < best_derr ) )
            {
                second_err = best_err;
                best = id;
                best_err = err;
                best_derr = derr;
            }
            else if ( err < second_err )
            {
                second_err = err;
            }
        }

        if ( best < 0 )
        {
            ++n_unmatched_;
            continue;
        }

        assign( best, it, "nearest_dir", best_err, ( second_err - best_err ) < 1.0 );
    }
}

/*-------------------------------------------------------------------*/
/*!
  match the world model player tracks to the true objects
 */
void
DenoiseLogger::Impl::matchTracks()
{
    for ( TrackAsg & a : trk_ ) a = TrackAsg();
    extra_.clear();

    const WorldModel & wm = agent_.world();

    struct Cand {
        const PlayerObject * o_;
        const char * list_;
    };

    std::vector< Cand > all;
    for ( const PlayerObject & o : wm.teammateObjects() ) all.push_back( Cand{ &o, "teammate" } );
    for ( const PlayerObject & o : wm.opponentObjects() ) all.push_back( Cand{ &o, "opponent" } );
    for ( const PlayerObject & o : wm.unknownPlayerObjects() ) all.push_back( Cand{ &o, "unknown" } );

    auto add_extra = [this]( const PlayerObject * o, const char * list, const char * reason )
        {
            ExtraTrack e;
            e.obj_ = o;
            e.list_ = list;
            e.reason_ = reason;
            extra_.push_back( e );
        };

    std::vector< Cand > unknown_unum;

    for ( const Cand & c : all )
    {
        if ( c.o_->isSelf() )
        {
            add_extra( c.o_, c.list_, "is_self" );
            continue;
        }

        if ( c.o_->unum() > 0 && c.o_->side() != NEUTRAL )
        {
            const int id = object_id( c.o_->side(), c.o_->unum() );
            if ( id < 1 || id == self_id_ || ( truth_valid_ && ! truth_[id].has_ ) )
            {
                add_extra( c.o_, c.list_, id == self_id_ ? "is_self" : "bad_unum" );
                continue;
            }

            // without the truth the track is still bound to its slot (distance is empty)
            const double dist = ( truth_valid_ && c.o_->posValid()
                                  ? g( c.o_->pos() ).dist( truth_[id].pos_ )
                                  : std::numeric_limits< double >::quiet_NaN() );

            if ( trk_[id].obj_ )
            {
                // duplicated unum. keep the track which has the smaller position count
                if ( c.o_->posCount() < trk_[id].obj_->posCount() )
                {
                    add_extra( trk_[id].obj_, trk_[id].list_, "dup_unum" );
                    trk_[id].obj_ = c.o_;
                    trk_[id].list_ = c.list_;
                    trk_[id].method_ = "unum";
                    trk_[id].dist_ = dist;
                }
                else
                {
                    add_extra( c.o_, c.list_, "dup_unum" );
                }
                continue;
            }

            trk_[id].obj_ = c.o_;
            trk_[id].list_ = c.list_;
            trk_[id].method_ = "unum";
            trk_[id].dist_ = dist;
            continue;
        }

        if ( ! truth_valid_ )
        {
            add_extra( c.o_, c.list_, "no_truth" );
            continue;
        }

        unknown_unum.push_back( c );
    }

    if ( ! truth_valid_ ) return;

    // greedy nearest matching of the tracks without a uniform number
    struct Pair {
        double dist_;
        std::size_t index_;
        int id_;
    };

    std::vector< Pair > pairs;
    for ( std::size_t i = 0; i < unknown_unum.size(); ++i )
    {
        const PlayerObject * o = unknown_unum[i].o_;
        if ( ! o->posValid() ) continue;

        const Vector2D pos = g( o->pos() );
        for ( int id = 1; id <= 22; ++id )
        {
            if ( id == self_id_ || trk_[id].obj_ || ! truth_[id].has_ ) continue;
            if ( o->side() != NEUTRAL && truth_[id].side_ != o->side() ) continue;

            const double d = pos.dist( truth_[id].pos_ );
            if ( d <= 5.0 ) pairs.push_back( Pair{ d, i, id } );
        }
    }

    std::sort( pairs.begin(), pairs.end(),
               []( const Pair & lhs, const Pair & rhs )
                 {
                     return lhs.dist_ < rhs.dist_;
                 } );

    std::vector< bool > assigned( unknown_unum.size(), false );
    for ( const Pair & p : pairs )
    {
        if ( assigned[p.index_] || trk_[p.id_].obj_ ) continue;

        assigned[p.index_] = true;
        trk_[p.id_].obj_ = unknown_unum[p.index_].o_;
        trk_[p.id_].method_ = "nearest";
        trk_[p.id_].dist_ = p.dist_;
    }

    for ( std::size_t i = 0; i < unknown_unum.size(); ++i )
    {
        if ( assigned[i] ) continue;
        add_extra( unknown_unum[i].o_, unknown_unum[i].list_,
                   unknown_unum[i].o_->posValid() ? "unmatched" : "no_pos" );
    }
}

/*-------------------------------------------------------------------*/
/*!
  build the part of the agent_cycle row which is known before the decision
 */
void
DenoiseLogger::Impl::buildCyclePrefix()
{
    row_cycle_.reset( new Row( SCHEMA_AGENT_CYCLE ) );
    Row & r = *row_cycle_;

    const WorldModel & wm = agent_.world();
    const SelfObject & self = wm.self();
    const GameTime & t = cur_;

    keys( r, t );
    r.b( "decision_made", true );

    // game state
    r.s( "game_mode", wm.gameMode().toCString() )
        .i( "score_l", wm.gameMode().scoreLeft() )
        .i( "score_r", wm.gameMode().scoreRight() );

    // timing
    const bool sb_received = ( agent_.bodySensor().time() == t );
    r.b( "sb_received", sb_received )
        .b( "see_received", agent_.visualSensor().time() == t )
        .b( "see_used", wm.seeTime() == t )
        .b( "fullstate_received", agent_.fullstateSensor().time() == t )
        .b( "truth_valid", truth_valid_ )
        .s( "decision_trigger", trigger_ );

    {
        const auto it = see_counts_.find( std::make_pair( t.cycle(), t.stopped() ) );
        r.i( "see_count_this_cycle", it == see_counts_.end() ? 0 : it->second );
    }

    if ( sb_received && last_sb_cycle_ == t.cycle() && last_sb_recv_ms_ > 0 )
    {
        r.i( "msec_sb_to_decision", DenoiseLogger::nowMsec() - last_sb_recv_ms_ );
    }

    // view
    r.s( "view_width", view_width_name( self.viewWidth().type() ) )
        .s( "view_quality", view_quality_name( self.viewQuality().type() ) )
        .d( "view_angle_deg", ServerParam::i().fullView() ? 360.0 : self.viewWidth().width() )
        .d( "focus_dist", self.focusDist() )
        .d( "focus_dir", self.focusDir().degree() )
        .i( "cycles_till_next_see", agent_.seeState().cyclesTillNextSee() );

    if ( agent_.visualSensor().time() == t )
    {
        r.i( "see_n_players", n_players_ )
            .i( "see_n_unmatched", n_unmatched_ );
    }

    // raw sense_body
    if ( sb_received )
    {
        const BodySensor & sb = agent_.bodySensor();
        r.s( "sb_view_quality", view_quality_name( sb.viewQuality().type() ) )
            .s( "sb_view_width", view_width_name( sb.viewWidth().type() ) )
            .d( "sb_stamina", sb.stamina() )
            .d( "sb_effort", sb.effort() )
            .d( "sb_stamina_capacity", sb.staminaCapacity() )
            .d( "sb_speed_mag", sb.speedMag() )
            .d( "sb_speed_dir", sb.speedDir() )
            .d( "sb_neck_dir", sb.neckDir() )
            .i( "sb_kick_count", sb.kickCount() )
            .i( "sb_dash_count", sb.dashCount() )
            .i( "sb_turn_count", sb.turnCount() )
            .i( "sb_say_count", sb.sayCount() )
            .i( "sb_turn_neck_count", sb.turnNeckCount() )
            .i( "sb_catch_count", sb.catchCount() )
            .i( "sb_move_count", sb.moveCount() )
            .i( "sb_change_view_count", sb.changeViewCount() )
            .i( "sb_change_focus_count", sb.changeFocusCount() )
            .i( "sb_arm_movable", sb.armMovable() )
            .i( "sb_arm_expires", sb.armExpires() )
            .d( "sb_pointto_dist", sb.pointtoDist() )
            .d( "sb_pointto_dir", sb.pointtoDir() )
            .i( "sb_pointto_count", sb.pointtoCount() )
            .s( "sb_attentionto_side", sb.attentiontoSide() == NEUTRAL
                ? std::string( "none" ) : std::string( 1, side_char( sb.attentiontoSide() ) ) )
            .i( "sb_attentionto_unum", sb.attentiontoUnum() )
            .i( "sb_attentionto_count", sb.attentiontoCount() )
            .i( "sb_tackle_expires", sb.tackleExpires() )
            .i( "sb_tackle_count", sb.tackleCount() )
            .b( "sb_coll_ball", sb.ballCollided() )
            .b( "sb_coll_player", sb.playerCollided() )
            .b( "sb_coll_post", sb.postCollided() )
            .i( "sb_charged_expires", sb.chargedExpires() )
            .s( "sb_card", card_name( sb.card() ) )
            .d( "sb_focus_dist", sb.focusDist() )
            .d( "sb_focus_dir", sb.focusDir() );
    }

    // estimate of the agent itself (global frame)
    if ( self.posValid() )
    {
        r.xy( "est_pos_x", "est_pos_y", g( self.pos() ) )
            .d( "est_pos_err_x", std::fabs( self.posError().x ) )
            .d( "est_pos_err_y", std::fabs( self.posError().y ) )
            .i( "est_pos_count", self.posCount() );
    }
    if ( self.velValid() )
    {
        r.xy( "est_vel_x", "est_vel_y", g( self.vel() ) )
            .i( "est_vel_count", self.velCount() );
    }
    r.d( "est_body", ga( self.body().degree() ) )
        .d( "est_neck", self.neck().degree() )
        .d( "est_stamina", self.stamina() )
        .d( "est_effort", self.effort() )
        .d( "est_recovery", self.recovery() )
        .d( "est_stamina_capacity", self.staminaCapacity() )
        .b( "est_kickable", self.isKickable() );
    if ( self.faceValid() )
    {
        r.d( "est_face", ga( self.face().degree() ) )
            .d( "est_face_err", self.faceError() )
            .i( "est_face_count", self.faceCount() );
    }

    // raw self localization output (before blending). only if it was computed at this cycle.
    if ( wm.selfRawTime() == t )
    {
        if ( wm.selfRawPos().isValid() )
        {
            r.xy( "rawfix_pos_x", "rawfix_pos_y", g( wm.selfRawPos() ) )
                .d( "rawfix_pos_err_x", std::fabs( wm.selfRawPosError().x ) )
                .d( "rawfix_pos_err_y", std::fabs( wm.selfRawPosError().y ) );
        }
        if ( wm.selfRawFaceValid() )
        {
            r.d( "rawfix_face", ga( wm.selfRawFace() ) )
                .d( "rawfix_face_err", wm.selfRawFaceError() );
        }
    }

    // ground truth of the agent itself
    if ( truth_valid_ )
    {
        const TruthObj & o = truth_[self_id_];
        r.xy( "tru_pos_x", "tru_pos_y", o.pos_, PREC_TRUE )
            .xy( "tru_vel_x", "tru_vel_y", o.vel_, PREC_TRUE )
            .d( "tru_body", o.body_, PREC_TRUE )
            .d( "tru_neck", o.neck_, PREC_TRUE )
            .d( "tru_stamina", o.stamina_ )
            .d( "tru_effort", o.effort_ )
            .d( "tru_recovery", o.recovery_ )
            .d( "tru_stamina_capacity", o.capacity_ )
            .d( "tru_focus_dist", o.focus_dist_ )
            .d( "tru_focus_dir", o.focus_dir_ )
            .b( "tru_kicked", o.kicked_ )
            .b( "tru_tackle", o.tackle_ )
            .b( "tru_charged", o.charged_ )
            .s( "tru_card", card_name( o.card_ ) )
            .b( "tru_goalie", o.goalie_ );
        if ( o.pointto_dist_ >= 0.0 )
        {
            r.d( "tru_pointto_dist", o.pointto_dist_ )
                .d( "tru_pointto_dir", o.pointto_dir_ );
        }
        if ( o.type_ >= 0 ) r.i( "tru_player_type", o.type_ );

        // noise = observed - truth. the truth is the one of this cycle.
        const double tru_face = o.body_ + o.neck_;
        if ( self.posValid() )
        {
            r.dxy( "n_est_pos_dx", "n_est_pos_dy", "n_est_pos_d", g( self.pos() ), o.pos_ );
        }
        if ( self.velValid() )
        {
            r.dxy( "n_est_vel_dx", "n_est_vel_dy", "n_est_vel_d", g( self.vel() ), o.vel_ );
        }
        // neck and stamina have no validity flag (neck is the forward model + sense_body)
        r.nang( "n_est_neck", self.neck().degree(), o.neck_ )
            .nd( "n_est_stamina", self.stamina(), o.stamina_ );
        // body and face share the validity of faceValid()
        if ( self.faceValid() )
        {
            r.nang( "n_est_body", ga( self.body().degree() ), o.body_ )
                .nang( "n_est_face", ga( self.face().degree() ), tru_face );
        }
        if ( wm.selfRawTime() == t )
        {
            if ( wm.selfRawPos().isValid() )
            {
                r.dxy( "n_rawfix_pos_dx", "n_rawfix_pos_dy", "n_rawfix_pos_d", g( wm.selfRawPos() ), o.pos_ );
            }
            if ( wm.selfRawFaceValid() )
            {
                r.nang( "n_rawfix_face", ga( wm.selfRawFace() ), tru_face );
            }
        }
        if ( sb_received )
        {
            const BodySensor & sb = agent_.bodySensor();
            const double tru_speed = ( o.vel_.isValid() ? o.vel_.r() : std::numeric_limits< double >::quiet_NaN() );
            r.nd( "n_sb_speed_mag", sb.speedMag(), tru_speed )
                .nd( "n_sb_neck", sb.neckDir(), o.neck_ )
                .nd( "n_sb_stamina", sb.stamina(), o.stamina_ );
            // the direction of a zero velocity is not defined
            if ( std::isfinite( tru_speed ) && tru_speed > 1.0e-6 )
            {
                r.nang( "n_sb_speed_dir", sb.speedDir(), o.vel_.th().degree() - tru_face );
            }
        }
    }
}

/*-------------------------------------------------------------------*/
void
DenoiseLogger::Impl::writeObjects()
{
    // all the 22 rows are always written. the truth columns are empty if the truth is not valid.
    if ( self_id_ < 1 || 22 < self_id_ ) return;

    const WorldModel & wm = agent_.world();
    const TruthObj & me = truth_[self_id_];
    const double view_width = ( ServerParam::i().fullView() ? 360.0 : wm.self().viewWidth().width() );
    const double rad2deg = 180.0 / 3.14159265358979323846;

    for ( int id = 0; id <= 22; ++id )
    {
        if ( id == self_id_ ) continue;

        const TruthObj & o = truth_[id];

        Row r( SCHEMA_OBJECTS );
        keys( r, cur_ );
        r.i( "object_id", id );

        if ( id > 0 )
        {
            const SideID side = ( id <= 11 ? LEFT : RIGHT );
            r.s( "obj_side", std::string( 1, side_char( side ) ) )
                .i( "obj_unum", id <= 11 ? id : id - 11 )
                .b( "is_teammate", side == side_ );
            if ( o.has_ )
            {
                r.b( "is_goalie", o.goalie_ );
                if ( o.type_ >= 0 ) r.i( "player_type", o.type_ );
            }
        }

        r.b( "truth_valid", truth_valid_ );
        if ( truth_valid_ ) r.b( "tru_on_field", o.has_ );

        // true relative geometry of this object. used by the noise columns.
        bool have_rel = false;
        double t_dist = 0.0, t_dir = 0.0;
        double t_dch = std::numeric_limits< double >::quiet_NaN();
        double t_dirch = std::numeric_limits< double >::quiet_NaN();
        const bool tru_obj = ( truth_valid_ && o.has_ );
        if ( truth_valid_ && o.has_ )
        {
            r.xy( "tru_pos_x", "tru_pos_y", o.pos_, PREC_TRUE )
                .xy( "tru_vel_x", "tru_vel_y", o.vel_, PREC_TRUE );
            if ( id > 0 )
            {
                r.d( "tru_body", o.body_, PREC_TRUE )
                    .d( "tru_neck", o.neck_, PREC_TRUE )
                    .d( "tru_stamina", o.stamina_ )
                    .d( "tru_effort", o.effort_ )
                    .d( "tru_recovery", o.recovery_ )
                    .d( "tru_stamina_capacity", o.capacity_ )
                    .b( "tru_kicked", o.kicked_ )
                    .b( "tru_tackle", o.tackle_ )
                    .b( "tru_charged", o.charged_ )
                    .s( "tru_card", card_name( o.card_ ) );
            }

            double dist = 0.0, dir = 0.0, fdist = 0.0;
            if ( relTo( o.pos_, &dist, &dir, &fdist ) )
            {
                have_rel = true;
                t_dist = dist;
                t_dir = dir;
                r.d( "true_dist", dist, PREC_TRUE )
                    .d( "true_dir", dir, PREC_TRUE )
                    .d( "true_focus_dist", fdist, PREC_TRUE )
                    .b( "in_cone", ServerParam::i().fullView() || std::fabs( dir ) <= view_width * 0.5 )
                    .b( "within_visible_dist", dist <= ServerParam::i().visibleDistance() );

                if ( dist > 1.0e-6 )
                {
                    const Vector2D rv = o.vel_ - me.vel_;
                    const Vector2D u = ( o.pos_ - me.pos_ ) / dist;
                    t_dch = rv.x * u.x + rv.y * u.y;
                    t_dirch = ( u.x * rv.y - u.y * rv.x ) / dist * rad2deg;
                    r.d( "true_dch", t_dch, PREC_TRUE )
                        .d( "true_dirch", t_dirch, PREC_TRUE );
                }
            }
        }

        // raw see
        const bool see_received = ( agent_.visualSensor().time() == cur_ );
        if ( see_received )
        {
            if ( id == 0 )
            {
                if ( seen_ball_ )
                {
                    const VisualSensor::BallT & b = seen_ball_data_;
                    r.b( "seen", true )
                        .s( "form", b.has_vel_ ? "full" : "no_vel" )
                        .d( "dist", b.dist_ )
                        .d( "dir", b.dir_ );
                    if ( b.has_vel_ )
                    {
                        r.d( "dist_chg", b.dist_chng_ ).d( "dir_chg", b.dir_chng_ );
                    }
                    if ( have_rel )
                    {
                        r.nd( "n_dist", b.dist_, t_dist )
                            .nang( "n_dir", b.dir_, t_dir );
                        if ( t_dist > 1.0e-6 ) r.nd( "n_dist_rel", ( b.dist_ - t_dist ) / t_dist, 0.0 );
                        if ( b.has_vel_ )
                        {
                            r.nd( "n_dist_chg", b.dist_chng_, t_dch )
                                .nd( "n_dir_chg", b.dir_chng_, t_dirch );
                        }
                    }
                }
                else
                {
                    r.b( "seen", false );
                }
            }
            else
            {
                const SeenRec & s = seen_[id];
                r.b( "seen", s.has_ );
                if ( s.has_ )
                {
                    const VisualSensor::PlayerT & p = s.p_;
                    r.s( "form", s.kind_ == 0 ? ( p.has_vel_ ? "full" : "no_vel" )
                         : s.kind_ == 1 ? "team_only" : "anonymous" )
                        .b( "team_visible", s.kind_ != 2 )
                        .b( "unum_visible", s.kind_ == 0 )
                        .d( "dist", p.dist_ )
                        .d( "dir", p.dir_ )
                        .s( "match_method", s.method_ )
                        .d( "match_err_deg", s.err_, PREC_TRUE )
                        .b( "match_ambiguous", s.amb_ )
                        .i( "raw_dup_dropped", s.dup_ )
                        .b( "kick_flag", p.kicking_ )
                        .b( "tackle_flag", p.tackle_ );
                    if ( p.has_vel_ )
                    {
                        r.d( "dist_chg", p.dist_chng_ ).d( "dir_chg", p.dir_chng_ );
                    }
                    if ( p.body_ != VisualSensor::DIR_ERR ) r.d( "body_rel", p.body_ );
                    if ( p.face_ != VisualSensor::DIR_ERR ) r.d( "head_rel", p.face_ );
                    if ( p.arm_ != VisualSensor::DIR_ERR ) r.d( "pointto_rel", p.arm_ );

                    // noise of the raw see
                    if ( have_rel )
                    {
                        r.nd( "n_dist", p.dist_, t_dist )
                            .nang( "n_dir", p.dir_, t_dir );
                        if ( t_dist > 1.0e-6 ) r.nd( "n_dist_rel", ( p.dist_ - t_dist ) / t_dist, 0.0 );
                        if ( p.has_vel_ )
                        {
                            r.nd( "n_dist_chg", p.dist_chng_, t_dch )
                                .nd( "n_dir_chg", p.dir_chng_, t_dirch );
                        }
                    }
                    if ( tru_obj )
                    {
                        // server convention (visualsenderplayer.h calcBodyDir / calcHeadDir)
                        const double ref = me.body_ + me.neck_;
                        if ( p.body_ != VisualSensor::DIR_ERR ) r.nang( "n_body_rel", p.body_, o.body_ - ref );
                        if ( p.face_ != VisualSensor::DIR_ERR ) r.nang( "n_head_rel", p.face_, o.body_ + o.neck_ - ref );
                    }

                    // localization record of this raw entry (made by this cycle's see)
                    r.b( "ls_valid", s.rec_has_ );
                    if ( s.rec_has_ )
                    {
                        const WorldModel::SeePlayerRecord & lp = s.rec_;
                        r.b( "ls_rel_valid", lp.rpos_.isValid() )
                            .b( "ls_pos_valid", lp.pos_.isValid() )
                            .b( "ls_vel_valid", lp.vel_.isValid() )
                            .xy( "ls_rpos_x", "ls_rpos_y", g( lp.rpos_ ) )
                            .xy( "ls_pos_x", "ls_pos_y", g( lp.pos_ ) )
                            .xy( "ls_vel_x", "ls_vel_y", g( lp.vel_ ) )
                            .d( "ls_dist_error", lp.dist_error_ );
                        if ( tru_obj )
                        {
                            r.dxy( "n_ls_rpos_dx", "n_ls_rpos_dy", "n_ls_rpos_d", g( lp.rpos_ ), o.pos_ - me.pos_ )
                                .dxy( "n_ls_pos_dx", "n_ls_pos_dy", "n_ls_pos_d", g( lp.pos_ ), o.pos_ )
                                .dxy( "n_ls_vel_dx", "n_ls_vel_dy", "n_ls_vel_d", g( lp.vel_ ), o.vel_ );
                        }
                    }
                }
            }
        }

        // world model track
        if ( id == 0 )
        {
            const BallObject & b = wm.ball();
            r.b( "track_found", b.posValid() );
            if ( b.posValid() )
            {
                const Vector2D pos = g( b.pos() );
                r.s( "track_match", "ball" )
                    .d( "track_match_dist", o.has_ ? pos.dist( o.pos_ ) : std::numeric_limits< double >::quiet_NaN() )
                    .xy( "trk_pos_x", "trk_pos_y", pos )
                    .d( "trk_pos_err_x", std::fabs( b.posError().x ) )
                    .d( "trk_pos_err_y", std::fabs( b.posError().y ) )
                    .i( "trk_pos_count", b.posCount() )
                    .i( "trk_seen_pos_count", b.seenPosCount() );
                if ( b.velValid() )
                {
                    r.xy( "trk_vel_x", "trk_vel_y", g( b.vel() ) )
                        .d( "trk_vel_err_x", std::fabs( b.velError().x ) )
                        .d( "trk_vel_err_y", std::fabs( b.velError().y ) )
                        .i( "trk_vel_count", b.velCount() )
                        .i( "trk_seen_vel_count", b.seenVelCount() );
                }
                r.b( "trk_is_ghost", b.ghostCount() > 0 )
                    .i( "trk_ghost_count", b.ghostCount() )
                    .i( "trk_lost_count", b.lostCount() )
                    .d( "trk_dist_from_self", b.distFromSelf() )
                    .d( "trk_angle_from_self", ga( b.angleFromSelf().degree() ) );
                if ( tru_obj )
                {
                    r.dxy( "n_trk_pos_dx", "n_trk_pos_dy", "n_trk_pos_d", pos, o.pos_ );
                    if ( b.velValid() )
                    {
                        r.dxy( "n_trk_vel_dx", "n_trk_vel_dy", "n_trk_vel_d", g( b.vel() ), o.vel_ );
                    }
                }
            }
            // the ball initializes heard pos/vel to (0,0) with count 1000 (= never heard)
            const bool ball_heard_pos = ( b.heardPos().isValid() && b.heardPosCount() < 1000 );
            const bool ball_heard_vel = ( b.heardVel().isValid() && b.heardVelCount() < 1000 );
            if ( ball_heard_pos )
            {
                r.xy( "trk_heard_pos_x", "trk_heard_pos_y", g( b.heardPos() ) )
                    .i( "trk_heard_pos_count", b.heardPosCount() );
            }
            if ( ball_heard_vel )
            {
                r.xy( "trk_heard_vel_x", "trk_heard_vel_y", g( b.heardVel() ) )
                    .i( "trk_heard_vel_count", b.heardVelCount() );
            }
            if ( tru_obj )
            {
                if ( ball_heard_pos )
                {
                    r.dnorm( "n_trk_heard_pos_d", g( b.heardPos() ), o.pos_ );
                }
                if ( ball_heard_vel )
                {
                    r.dnorm( "n_trk_heard_vel_d", g( b.heardVel() ), o.vel_ );
                }
            }

            // localized raw see of the ball. the write-only record of the world model is used
            // only if it was made by this cycle's see (never inferred from the *Count() values).
            if ( see_received )
            {
                const WorldModel::SeeBallRecord & lb = wm.seeBallRecord();
                const bool rec = ( lb.time_ == cur_ );
                r.b( "ls_valid", rec && ( lb.rpos_valid_ || lb.pos_valid_ || lb.vel_valid_ ) );
                if ( rec )
                {
                    r.b( "ls_rel_valid", lb.rpos_valid_ )
                        .b( "ls_pos_valid", lb.pos_valid_ )
                        .b( "ls_vel_valid", lb.vel_valid_ )
                        .b( "ls_rvel_valid", lb.rvel_valid_ );
                    if ( lb.rpos_valid_ )
                    {
                        r.xy( "ls_rpos_x", "ls_rpos_y", g( lb.rpos_ ) )
                            .d( "ls_rpos_err_x", std::fabs( lb.rpos_err_.x ) )
                            .d( "ls_rpos_err_y", std::fabs( lb.rpos_err_.y ) );
                    }
                    if ( lb.rvel_valid_ )
                    {
                        r.xy( "ls_rvel_x", "ls_rvel_y", g( lb.rvel_ ) )
                            .d( "ls_rvel_err_x", std::fabs( lb.rvel_err_.x ) )
                            .d( "ls_rvel_err_y", std::fabs( lb.rvel_err_.y ) );
                    }
                    if ( lb.pos_valid_ )
                    {
                        r.xy( "ls_pos_x", "ls_pos_y", g( lb.pos_ ) )
                            .d( "ls_pos_err_x", std::fabs( lb.pos_err_.x ) )
                            .d( "ls_pos_err_y", std::fabs( lb.pos_err_.y ) );
                    }
                    if ( lb.vel_valid_ )
                    {
                        r.xy( "ls_vel_x", "ls_vel_y", g( lb.vel_ ) )
                            .d( "ls_vel_err_x", std::fabs( lb.vel_err_.x ) )
                            .d( "ls_vel_err_y", std::fabs( lb.vel_err_.y ) )
                            .i( "ls_vel_count", lb.vel_count_ );
                    }
                    if ( tru_obj )
                    {
                        // rvel is the ball velocity relative to the observer: v_ball - v_self
                        r.dxy( "n_ls_rpos_dx", "n_ls_rpos_dy", "n_ls_rpos_d",
                               lb.rpos_valid_ ? g( lb.rpos_ ) : Vector2D( Vector2D::INVALIDATED ), o.pos_ - me.pos_ )
                            .dxy( "n_ls_pos_dx", "n_ls_pos_dy", "n_ls_pos_d",
                                  lb.pos_valid_ ? g( lb.pos_ ) : Vector2D( Vector2D::INVALIDATED ), o.pos_ )
                            .dxy( "n_ls_vel_dx", "n_ls_vel_dy", "n_ls_vel_d",
                                  lb.vel_valid_ ? g( lb.vel_ ) : Vector2D( Vector2D::INVALIDATED ), o.vel_ )
                            .dxy( "n_ls_rvel_dx", "n_ls_rvel_dy", "n_ls_rvel_d",
                                  lb.rvel_valid_ ? g( lb.rvel_ ) : Vector2D( Vector2D::INVALIDATED ), o.vel_ - me.vel_ );
                    }
                }
            }
        }
        else
        {
            const TrackAsg & a = trk_[id];
            r.b( "track_found", a.obj_ != nullptr );
            if ( a.obj_ )
            {
                const PlayerObject * p = a.obj_;
                r.s( "track_match", a.method_ )
                    .d( "track_match_dist", a.dist_ )
                    .b( "trk_unum_known", p->unum() > 0 );
                if ( p->posValid() )
                {
                    r.xy( "trk_pos_x", "trk_pos_y", g( p->pos() ) )
                        .i( "trk_pos_count", p->posCount() )
                        .i( "trk_seen_pos_count", p->seenPosCount() );
                }
                if ( p->velValid() )
                {
                    r.xy( "trk_vel_x", "trk_vel_y", g( p->vel() ) )
                        .i( "trk_vel_count", p->velCount() )
                        .i( "trk_seen_vel_count", p->seenVelCount() );
                }
                if ( p->bodyValid() )
                {
                    r.d( "trk_body", ga( p->body().degree() ) ).i( "trk_body_count", p->bodyCount() );
                }
                if ( p->faceValid() )
                {
                    r.d( "trk_face", ga( p->face().degree() ) ).i( "trk_face_count", p->faceCount() );
                }
                r.b( "trk_is_ghost", p->isGhost() )
                    .i( "trk_ghost_count", p->ghostCount() )
                    .d( "trk_dist_from_self", p->distFromSelf() )
                    .d( "trk_angle_from_self", ga( p->angleFromSelf().degree() ) );
                if ( p->heardPos().isValid() )
                {
                    r.xy( "trk_heard_pos_x", "trk_heard_pos_y", g( p->heardPos() ) )
                        .i( "trk_heard_pos_count", p->heardPosCount() );
                }
                if ( tru_obj )
                {
                    if ( p->posValid() )
                    {
                        r.dxy( "n_trk_pos_dx", "n_trk_pos_dy", "n_trk_pos_d", g( p->pos() ), o.pos_ );
                    }
                    if ( p->velValid() )
                    {
                        r.dxy( "n_trk_vel_dx", "n_trk_vel_dy", "n_trk_vel_d", g( p->vel() ), o.vel_ );
                    }
                    if ( p->bodyValid() ) r.nang( "n_trk_body", ga( p->body().degree() ), o.body_ );
                    if ( p->faceValid() ) r.nang( "n_trk_face", ga( p->face().degree() ), o.body_ + o.neck_ );
                    if ( p->heardPos().isValid() )
                    {
                        r.dnorm( "n_trk_heard_pos_d", g( p->heardPos() ), o.pos_ );
                    }
                }
            }
        }

        t_obj_.write( r );
    }
}

/*-------------------------------------------------------------------*/
void
DenoiseLogger::Impl::writeLandmarks()
{
    if ( ! t_land_.isOpen() ) return;

    const VisualSensor & vs = agent_.visualSensor();
    if ( vs.time() != cur_ ) return;

    auto emit = [this]( const std::string & name,
                        const int marker_id,
                        const char * kind,
                        const VisualSensor::PolarT & polar )
        {
            Row r( SCHEMA_LANDMARKS );
            keys( r, cur_ );
            r.s( "landmark_name", name ).s( "kind", kind );
            if ( marker_id >= 0 ) r.i( "landmark_id", marker_id );
            r.d( "dist", polar.dist_ ).d( "dir", polar.dir_ );

            if ( marker_id >= 0 )
            {
                const auto it = object_table_.landmarkMap().find( static_cast< MarkerID >( marker_id ) );
                if ( it != object_table_.landmarkMap().end() )
                {
                    // the landmark map is defined in the global frame. no reversing is needed.
                    r.xy( "land_x", "land_y", it->second );

                    double d = 0.0, dir = 0.0, fd = 0.0;
                    if ( relTo( it->second, &d, &dir, &fd ) )
                    {
                        r.d( "true_dist", d, PREC_TRUE )
                            .d( "true_dir", dir, PREC_TRUE )
                            .d( "true_focus_dist", fd, PREC_TRUE );
                        // noise of the raw see = observed - truth
                        r.nd( "n_dist", polar.dist_, d )
                            .nang( "n_dir", polar.dir_, dir );
                        if ( d > 1.0e-6 ) r.nd( "n_dist_rel", ( polar.dist_ - d ) / d, 0.0 );
                    }
                }
            }

            t_land_.write( r );
        };

    for ( const VisualSensor::MarkerT & m : vs.markers() )
    {
        const bool goal = ( m.object_type_ == VisualSensor::Obj_Goal );
        const int id = static_cast< int >( m.id_ );
        const bool valid = ( 0 <= id && id < MARKER_NAME_SIZE );
        emit( valid ? MARKER_NAMES[id] : "unknown", valid ? id : -1, goal ? "goal" : "flag", m );
    }

    for ( const VisualSensor::MarkerT & m : vs.behindMarkers() )
    {
        const bool goal = ( m.object_type_ == VisualSensor::Obj_Goal_Behind );
        emit( "behind", -1, goal ? "behind_goal" : "behind_flag", m );
    }

    for ( const VisualSensor::LineT & l : vs.lines() )
    {
        const char * name = ( l.id_ == Line_Left ? "Line_Left"
                              : l.id_ == Line_Right ? "Line_Right"
                              : l.id_ == Line_Top ? "Line_Top"
                              : l.id_ == Line_Bottom ? "Line_Bottom"
                              : "Line_Unknown" );
        emit( name, -1, "line", l );
    }
}

/*-------------------------------------------------------------------*/
void
DenoiseLogger::Impl::writeExtraTracks()
{
    for ( const ExtraTrack & e : extra_ )
    {
        const PlayerObject * p = e.obj_;

        Row r( SCHEMA_EXTRA_TRACKS );
        keys( r, cur_ );
        r.s( "track_list", e.list_ ).s( "reason", e.reason_ );
        if ( p->side() != NEUTRAL ) r.s( "trk_side", std::string( 1, side_char( p->side() ) ) );
        if ( p->unum() > 0 ) r.i( "trk_unum", p->unum() );
        if ( p->posValid() ) r.xy( "pos_x", "pos_y", g( p->pos() ) ).i( "pos_count", p->posCount() );
        if ( p->velValid() ) r.xy( "vel_x", "vel_y", g( p->vel() ) ).i( "vel_count", p->velCount() );
        if ( p->bodyValid() ) r.d( "body", ga( p->body().degree() ) ).i( "body_count", p->bodyCount() );
        if ( p->faceValid() ) r.d( "face", ga( p->face().degree() ) ).i( "face_count", p->faceCount() );
        r.b( "is_ghost", p->isGhost() )
            .d( "dist_from_self", p->distFromSelf() )
            .d( "angle_from_self", ga( p->angleFromSelf().degree() ) );
        t_extra_.write( r );
    }
}

/*-------------------------------------------------------------------*/
std::string
DenoiseLogger::Impl::decodeHear( const PendingHear & h ) const
{
    if ( h.kind_ != "teammate" || h.sender_unum_ < 1 ) return std::string();

    const AudioMemory & am = agent_.world().audioMemory();
    const GameTime & t = h.time_;
    const int s = h.sender_unum_;
    const double sg = sgn();

    std::vector< std::string > items;

    if ( am.ballTime() == t )
        for ( const AudioMemory::Ball & e : am.ball() )
            if ( e.sender_ == s )
                items.push_back( "ball(" + vec_text( g( e.pos_ ) ) + " " + vec_text( g( e.vel_ ) ) + ")" );

    if ( am.passTime() == t )
        for ( const AudioMemory::Pass & e : am.pass() )
            if ( e.sender_ == s )
                items.push_back( "pass(" + std::to_string( e.receiver_ ) + " " + vec_text( g( e.receive_pos_ ) ) + ")" );

    if ( am.goalieTime() == t )
        for ( const AudioMemory::Goalie & e : am.goalie() )
            if ( e.sender_ == s )
                items.push_back( "goalie(" + vec_text( g( e.pos_ ) ) + " " + fmt_double( ga( e.body_.degree() ), PREC ) + ")" );

    if ( am.playerTime() == t )
        for ( const AudioMemory::Player & e : am.player() )
            if ( e.sender_ == s )
                items.push_back( "player(" + std::to_string( e.unum_ ) + " " + vec_text( g( e.pos_ ) )
                                 + " " + fmt_double( ga( e.body_ ), PREC ) + " " + fmt_double( e.stamina_, PREC ) + ")" );

    if ( am.ourInterceptTime() == t )
        for ( const AudioMemory::OurIntercept & e : am.ourIntercept() )
            if ( e.sender_ == s )
                items.push_back( "our_intercept(" + std::to_string( e.interceptor_ ) + " " + std::to_string( e.cycle_ ) + ")" );

    if ( am.oppInterceptTime() == t )
        for ( const AudioMemory::OppIntercept & e : am.oppIntercept() )
            if ( e.sender_ == s )
                items.push_back( "opp_intercept(" + std::to_string( e.interceptor_ ) + " " + std::to_string( e.cycle_ ) + ")" );

    if ( am.offsideLineTime() == t )
        for ( const AudioMemory::OffsideLine & e : am.offsideLine() )
            if ( e.sender_ == s )
                items.push_back( "offside_line(" + fmt_double( e.x_ * sg, PREC ) + ")" );

    if ( am.defenseLineTime() == t )
        for ( const AudioMemory::DefenseLine & e : am.defenseLine() )
            if ( e.sender_ == s )
                items.push_back( "defense_line(" + fmt_double( e.x_ * sg, PREC ) + ")" );

    if ( am.passRequestTime() == t )
        for ( const AudioMemory::PassRequest & e : am.passRequest() )
            if ( e.sender_ == s )
                items.push_back( "pass_request(" + vec_text( g( e.pos_ ) ) + ")" );

    if ( am.runRequestTime() == t )
        for ( const AudioMemory::RunRequest & e : am.runRequest() )
            if ( e.sender_ == s )
                items.push_back( "run_request(" + std::to_string( e.runner_ ) + " " + vec_text( g( e.pos_ ) ) + ")" );

    if ( am.dribbleTime() == t )
        for ( const AudioMemory::Dribble & e : am.dribble() )
            if ( e.sender_ == s )
                items.push_back( "dribble(" + vec_text( g( e.target_ ) ) + " " + std::to_string( e.queue_count_ ) + ")" );

    if ( am.staminaTime() == t )
        for ( const AudioMemory::Stamina & e : am.stamina() )
            if ( e.sender_ == s )
                items.push_back( "stamina(" + fmt_double( e.rate_, PREC ) + ")" );

    if ( am.recoveryTime() == t )
        for ( const AudioMemory::Recovery & e : am.recovery() )
            if ( e.sender_ == s )
                items.push_back( "recovery(" + fmt_double( e.rate_, PREC ) + ")" );

    if ( am.staminaCapacityTime() == t )
        for ( const AudioMemory::StaminaCapacity & e : am.staminaCapacity() )
            if ( e.sender_ == s )
                items.push_back( "stamina_capacity(" + fmt_double( e.rate_, PREC ) + ")" );

    if ( am.waitRequestTime() == t )
        for ( const AudioMemory::WaitRequest & e : am.waitRequest() )
            if ( e.sender_ == s )
                items.push_back( "wait_request" );

    if ( am.setplayTime() == t )
        for ( const AudioMemory::Setplay & e : am.setplay() )
            if ( e.sender_ == s )
                items.push_back( "setplay(" + std::to_string( e.wait_step_ ) + ")" );

    if ( am.freeMessageTime() == t )
        for ( const AudioMemory::FreeMessage & e : am.freeMessage() )
            if ( e.sender_ == s )
                items.push_back( "free(" + e.message_ + ")" );

    return join( items, ';' );
}

/*-------------------------------------------------------------------*/
/*!
  noise of the decoded teammate payload (decoded - truth of this cycle).
  only if the message was heard at the current cycle. the content of a heard message may
  describe the sender's world of an earlier cycle, so a part of the noise is the aging.
 */
void
DenoiseLogger::Impl::hearNoise( Row & r,
                                const PendingHear & h ) const
{
    if ( ! truth_valid_ || h.time_ != cur_
         || h.kind_ != "teammate" || h.sender_unum_ < 1 )
    {
        return;
    }

    const AudioMemory & am = agent_.world().audioMemory();
    const GameTime & t = h.time_;
    const int s = h.sender_unum_;

    // the first item of the sender is used
    if ( am.ballTime() == t && truth_[0].has_ )
    {
        for ( const AudioMemory::Ball & e : am.ball() )
        {
            if ( e.sender_ != s ) continue;
            r.dxy( "n_ball_pos_dx", "n_ball_pos_dy", "n_ball_pos_d", g( e.pos_ ), truth_[0].pos_ )
                .dxy( "n_ball_vel_dx", "n_ball_vel_dy", "n_ball_vel_d", g( e.vel_ ), truth_[0].vel_ );
            break;
        }
    }

    // goalie item = the opponent goalie
    if ( am.goalieTime() == t )
    {
        int gid = -1;
        for ( int id = 1; id <= 22; ++id )
        {
            if ( truth_[id].has_ && truth_[id].goalie_ && truth_[id].side_ == opposite( side_ ) ) gid = id;
        }
        if ( gid > 0 )
        {
            for ( const AudioMemory::Goalie & e : am.goalie() )
            {
                if ( e.sender_ != s ) continue;
                r.dxy( "n_goalie_pos_dx", "n_goalie_pos_dy", "n_goalie_pos_d", g( e.pos_ ), truth_[gid].pos_ );
                break;
            }
        }
    }

    // player items: unum 1-11 = our team, 12-22 = opponent. summarized as count, mean and max of the position error.
    if ( am.playerTime() == t )
    {
        int n = 0;
        double sum = 0.0;
        double max_d = 0.0;
        for ( const AudioMemory::Player & e : am.player() )
        {
            if ( e.sender_ != s ) continue;
            const int id = ( 1 <= e.unum_ && e.unum_ <= 11 ? object_id( side_, e.unum_ )
                             : 12 <= e.unum_ && e.unum_ <= 22 ? object_id( opposite( side_ ), e.unum_ - 11 )
                             : -1 );
            if ( id < 1 || ! truth_[id].has_ ) continue;
            const Vector2D p = g( e.pos_ );
            if ( ! p.isValid() || ! truth_[id].pos_.isValid() ) continue;
            const double dist = p.dist( truth_[id].pos_ );
            if ( ! std::isfinite( dist ) ) continue;
            ++n;
            sum += dist;
            if ( dist > max_d ) max_d = dist;
        }
        if ( n > 0 )
        {
            r.i( "n_player_n", n )
                .d( "n_player_pos_d_mean", sum / n, PREC_TRUE )
                .d( "n_player_pos_d_max", max_d, PREC_TRUE );
        }
    }
}

/*-------------------------------------------------------------------*/
void
DenoiseLogger::Impl::addHear( const char * msg,
                              const GameTime & current )
{
    const std::string s = msg;

    PendingHear h;
    h.time_ = current;

    std::string head = s;
    const std::size_t q1 = s.find( '"' );
    if ( q1 != std::string::npos )
    {
        const std::size_t q2 = s.rfind( '"' );
        if ( q2 > q1 )
        {
            h.msg_ = s.substr( q1 + 1, q2 - q1 - 1 );
            h.has_content_ = true;
        }
        head = s.substr( 0, q1 );
    }

    for ( char & c : head )
    {
        if ( c == '(' || c == ')' ) c = ' ';
    }

    const std::vector< std::string > tok = split_ws( head );
    if ( tok.size() < 3 ) return;

    h.hear_cycle_ = std::atoi( tok[1].c_str() );

    const WorldModel & wm = agent_.world();
    const SideID our = wm.ourSide();

    auto is_number = []( const std::string & x )
        {
            if ( x.empty() ) return false;
            char * end = nullptr;
            std::strtod( x.c_str(), &end );
            return ( end != nullptr && *end == '\0' );
        };

    if ( tok[2] == "self" )
    {
        h.kind_ = "self";
        h.sender_side_ = our;
        h.sender_unum_ = wm.self().unum();
    }
    else if ( tok[2] == "our" )
    {
        h.kind_ = "teammate_partial";
        h.sender_side_ = our;
        h.sender_unum_ = ( tok.size() > 3 ? std::atoi( tok[3].c_str() ) : -1 );
    }
    else if ( tok[2] == "opp" )
    {
        h.kind_ = "opponent_partial";
        h.sender_side_ = opposite( our );
    }
    else if ( is_number( tok[2] ) && tok.size() > 3 && tok[3] == "our" )
    {
        h.kind_ = "teammate";
        h.sender_side_ = our;
        h.has_dir_ = true;
        h.dir_ = std::atof( tok[2].c_str() );
        h.sender_unum_ = ( tok.size() > 4 ? std::atoi( tok[4].c_str() ) : -1 );
    }
    else if ( is_number( tok[2] ) && tok.size() > 3 && tok[3] == "opp" )
    {
        h.kind_ = "opponent";
        h.sender_side_ = opposite( our );
        h.has_dir_ = true;
        h.dir_ = std::atof( tok[2].c_str() );
    }
    else
    {
        // referee, coach and trainer messages are only written to the raw stream
        return;
    }

    hear_pending_.push_back( h );
}

/*-------------------------------------------------------------------*/
void
DenoiseLogger::Impl::flushHear( const GameTime & t,
                                const bool all )
{
    std::vector< PendingHear > rest;

    for ( const PendingHear & h : hear_pending_ )
    {
        if ( ! all && h.hear_cycle_ > t.cycle() )
        {
            rest.push_back( h );
            continue;
        }

        Row r( SCHEMA_HEAR );
        keys( r, t );
        r.i( "hear_cycle", h.hear_cycle_ )
            .s( "sender_kind", h.kind_ )
            .b( "has_content", h.has_content_ );
        if ( h.sender_side_ != NEUTRAL ) r.s( "sender_side", std::string( 1, side_char( h.sender_side_ ) ) );
        if ( h.sender_unum_ > 0 ) r.i( "sender_unum", h.sender_unum_ );
        if ( h.has_dir_ ) r.d( "hear_dir", h.dir_ );
        if ( h.has_content_ ) r.s( "raw_msg", h.msg_ );

        const std::string decoded = decodeHear( h );
        if ( ! decoded.empty() ) r.s( "decoded", decoded );

        hearNoise( r, h );

        // true position of the sender
        if ( truth_valid_ && h.sender_unum_ > 0 && h.sender_side_ != NEUTRAL
             && h.kind_ != "opponent" && h.kind_ != "opponent_partial" )
        {
            const int id = object_id( h.sender_side_, h.sender_unum_ );
            if ( id >= 1 && truth_[id].has_ )
            {
                const TruthObj & o = truth_[id];
                r.xy( "sender_true_x", "sender_true_y", o.pos_, PREC_TRUE );
                double d = 0.0, dir = 0.0;
                if ( id != self_id_ && relTo( o.pos_, &d, &dir, nullptr ) )
                {
                    r.d( "sender_true_dist", d, PREC_TRUE ).d( "sender_true_dir_rel", dir, PREC_TRUE );
                }
            }
        }

        t_hear_.write( r );
    }

    hear_pending_.swap( rest );
}

/*-------------------------------------------------------------------*/
void
DenoiseLogger::Impl::addRaw( const char * dir,
                             const std::string & msg,
                             const long long ms,
                             const GameTime & t )
{
    if ( ! agent_.config().denoiseLogRaw() ) return;

    const int cycle = header_cycle( msg.c_str(), t.cycle() );

    if ( opened_ )
    {
        Row r( SCHEMA_RAW_STREAM );
        keys( r, GameTime( cycle, t.stopped() ) );
        r.i( "local_ms", ms ).s( "dir", dir ).s( "msg", raw_escape( msg ) );
        t_raw_.write( r );
    }
    else if ( raw_pending_.size() < MAX_RAW_PENDING )
    {
        PendingRaw p;
        p.dir_ = dir;
        p.msg_ = msg;
        p.ms_ = ms;
        p.cycle_ = cycle;
        p.stopped_ = t.stopped();
        raw_pending_.push_back( p );
    }
}

/*-------------------------------------------------------------------*/
void
DenoiseLogger::Impl::beginCycle( const GameTime & t,
                                 const std::string & trigger )
{
    // a cycle which was begun but never finished is written without commands
    if ( in_cycle_ )
    {
        endCycle( std::string() );
        if ( failed_ ) return;
    }

    cur_ = t;
    last_time_ = t;
    trigger_ = trigger;
    labels_.clear();
    pre_valid_ = false;
    in_cycle_ = true;

    // skipped cycles (no decision was made): placeholder rows in the same stopped == 0 stream.
    // late-arriving truth is never back-filled into these rows.
    if ( last_row_time_.stopped() == 0 && t.stopped() == 0
         && last_row_time_.cycle() >= 0 && t.cycle() - last_row_time_.cycle() > 1 )
    {
        const int first = last_row_time_.cycle() + 1;
        const int last = std::min( t.cycle() - 1, first + MAX_PLACEHOLDER_ROWS - 1 );
        for ( int c = first; c <= last; ++c )
        {
            Row r( SCHEMA_AGENT_CYCLE );
            keys( r, GameTime( c, 0 ) );
            r.b( "decision_made", false );
            t_cycle_.write( r );
        }
    }
    last_row_time_ = t;

    buildTruth( t );
    matchSeenPlayers( t );
    matchTracks();
    buildCyclePrefix();

    writeObjects();
    writeLandmarks();
    writeExtraTracks();
    flushHear( t, false );

    // forget the old see counters
    while ( ! see_counts_.empty() && see_counts_.begin()->first.first < t.cycle() - 10 )
    {
        see_counts_.erase( see_counts_.begin() );
    }

    if ( ! meta_written_ )
    {
        meta_written_ = true;
        if ( ! writeMeta( false ) ) fail( "failed to write meta.json" );
    }

    checkFailure();
}

/*-------------------------------------------------------------------*/
void
DenoiseLogger::Impl::endCycle( const std::string & batch )
{
    if ( ! in_cycle_ || ! row_cycle_ ) return;

    Row & r = *row_cycle_;
    const double sg = sgn();

    r.s( "cmd_raw", batch );

    std::vector< std::string > others;
    bool body_cmd_found = false;

    for ( const ParsedCommand & c : parse_commands( batch ) )
    {
        const std::vector< std::string > & a = c.args_;
        const auto num = [&a]( const std::size_t i ) -> double
            {
                return ( i < a.size() ? std::atof( a[i].c_str() ) : std::numeric_limits< double >::quiet_NaN() );
            };

        const bool body_cmd = ( c.name_ == "dash" || c.name_ == "turn" || c.name_ == "kick"
                                || c.name_ == "tackle" || c.name_ == "catch" || c.name_ == "move" );
        if ( body_cmd && body_cmd_found )
        {
            // only the first body command is written to the columns
            others.push_back( c.name_ );
            continue;
        }

        if ( c.name_ == "dash" )
        {
            r.s( "body_cmd", "dash" ).d( "dash_power", num( 0 ) ).d( "dash_dir", num( 1 ) );
        }
        else if ( c.name_ == "turn" )
        {
            r.s( "body_cmd", "turn" ).d( "turn_moment", num( 0 ) );
        }
        else if ( c.name_ == "kick" )
        {
            r.s( "body_cmd", "kick" ).d( "kick_power", num( 0 ) ).d( "kick_dir", num( 1 ) );
        }
        else if ( c.name_ == "tackle" )
        {
            r.s( "body_cmd", "tackle" ).d( "tackle_dir", num( 0 ) );
            if ( a.size() > 1 ) r.s( "tackle_foul", a[1] );
        }
        else if ( c.name_ == "catch" )
        {
            r.s( "body_cmd", "catch" ).d( "catch_dir", num( 0 ) );
        }
        else if ( c.name_ == "move" )
        {
            // the server multiplies the move position by the side. report it in the global frame.
            r.s( "body_cmd", "move" ).d( "move_x", num( 0 ) * sg ).d( "move_y", num( 1 ) * sg );
        }
        else if ( c.name_ == "turn_neck" )
        {
            r.d( "turn_neck_moment", num( 0 ) );
        }
        else if ( c.name_ == "change_view" )
        {
            r.s( "change_view", join( a, ' ' ) );
        }
        else if ( c.name_ == "change_focus" )
        {
            r.d( "change_focus_dist", num( 0 ) ).d( "change_focus_dir", num( 1 ) );
        }
        else if ( c.name_ == "say" )
        {
            r.s( "say_msg", c.quoted_ );
        }
        else if ( c.name_ == "pointto" )
        {
            if ( ! a.empty() && a[0] == "off" ) r.b( "pointto_off", true );
            else r.d( "pointto_dist", num( 0 ) ).d( "pointto_dir", num( 1 ) );
        }
        else if ( c.name_ == "attentionto" )
        {
            if ( ! a.empty() && a[0] == "off" ) r.b( "attentionto_off", true );
            else
            {
                if ( ! a.empty() ) r.s( "attentionto_side", a[0] );
                if ( a.size() > 1 ) r.d( "attentionto_unum", num( 1 ) );
            }
        }
        else
        {
            others.push_back( c.name_ );
        }

        if ( body_cmd ) body_cmd_found = true;
    }

    for ( const std::string & s : other_sent_ ) others.push_back( s );
    if ( ! others.empty() ) r.s( "other_sent", join( others, '|' ) );

    // predictions of the effector (snapshot taken by preSend() before makeCommand())
    if ( pre_valid_ )
    {
        if ( pre_self_pos_.isValid() ) r.xy( "next_self_pos_x", "next_self_pos_y", g( pre_self_pos_ ) );
        r.d( "next_self_body", ga( pre_self_body_ ) );

        if ( pre_ball_pos_.isValid() ) r.xy( "next_ball_pos_x", "next_ball_pos_y", g( pre_ball_pos_ ) );
        if ( pre_ball_vel_.isValid() ) r.xy( "next_ball_vel_x", "next_ball_vel_y", g( pre_ball_vel_ ) );

        r.s( "pred_next_view_width", view_width_name( pre_view_width_ ) )
            .i( "pred_next_see_cycles", pre_see_cycles_ );
    }
    pre_valid_ = false;

    if ( ! labels_.empty() ) r.s( "action_label", join( labels_, '|' ) );

    t_cycle_.write( r );

    row_cycle_.reset();
    in_cycle_ = false;
    other_sent_.clear();

    checkFailure();

    // the periodic flush is not crash-safe (a gz stream may lose the last block on a crash)
    if ( ++cycles_since_flush_ >= FLUSH_INTERVAL )
    {
        cycles_since_flush_ = 0;
        flushAll();
    }
}

/*-------------------------------------------------------------------*/
/*-------------------------------------------------------------------*/
/*-------------------------------------------------------------------*/
DenoiseLogger::DenoiseLogger( const PlayerAgent & agent )
    : M_impl( new Impl( agent ) )
{

}

/*-------------------------------------------------------------------*/
DenoiseLogger::~DenoiseLogger()
{
    // The owner agent may be partly destroyed here, so nothing of it is touched.
    // PlayerAgent calls close() explicitly while its members are alive.
    if ( M_impl && ! M_impl->closed_ )
    {
        M_impl->closeAll();
        M_impl->closed_ = true;
    }
}

/*-------------------------------------------------------------------*/
long long
DenoiseLogger::nowMsec()
{
    return std::chrono::duration_cast< std::chrono::milliseconds >
        ( std::chrono::system_clock::now().time_since_epoch() ).count();
}

/*-------------------------------------------------------------------*/
void
DenoiseLogger::onReceive( const char * msg,
                          const long long recv_msec,
                          const GameTime & current )
{
    Impl & m = *M_impl;
    if ( m.closed_ || m.open_failed_ || m.failed_ || ! msg ) return;

    // the output directory is opened as soon as the side and the uniform number are known
    m.ensureOpen();

    m.addRaw( "recv", msg, recv_msec, current );
    m.checkFailure();

    if ( ! std::strncmp( msg, "(see ", 5 ) )
    {
        // key = (cycle written in the see message, stopped counter of the analyzed message time)
        int t = 0;
        if ( std::sscanf( msg + 5, "%d", &t ) == 1 ) ++m.see_counts_[std::make_pair( t, current.stopped() )];
    }
    else if ( ! std::strncmp( msg, "(sense_body ", 12 ) )
    {
        int t = 0;
        if ( std::sscanf( msg + 12, "%d", &t ) == 1 )
        {
            m.last_sb_cycle_ = t;
            m.last_sb_recv_ms_ = recv_msec;
        }
    }
    else if ( ! std::strncmp( msg, "(hear ", 6 ) )
    {
        m.addHear( msg, current );
    }
}

/*-------------------------------------------------------------------*/
void
DenoiseLogger::onSend( const char * msg,
                       const bool is_action_batch,
                       const GameTime & current )
{
    Impl & m = *M_impl;
    if ( m.closed_ || m.open_failed_ || m.failed_ || ! msg ) return;

    m.ensureOpen();
    m.addRaw( "send", msg, nowMsec(), current );
    m.checkFailure();

    if ( ! is_action_batch )
    {
        for ( const ParsedCommand & c : parse_commands( msg ) )
        {
            m.other_sent_.push_back( c.name_ );
        }
    }
}

/*-------------------------------------------------------------------*/
void
DenoiseLogger::beginCycle( const GameTime & current,
                           const std::string & decision_trigger )
{
    Impl & m = *M_impl;
    if ( m.closed_ || m.open_failed_ || m.failed_ ) return;
    if ( ! m.ensureOpen() ) return;

    m.beginCycle( current, decision_trigger );
}

/*-------------------------------------------------------------------*/
void
DenoiseLogger::preSend()
{
    Impl & m = *M_impl;
    if ( ! m.opened_ || m.closed_ || m.failed_ || ! m.in_cycle_ ) return;

    m.snapshotPredictions();
}

/*-------------------------------------------------------------------*/
void
DenoiseLogger::endCycle( const std::string & command_batch )
{
    Impl & m = *M_impl;
    if ( ! m.opened_ || m.closed_ || m.failed_ ) return;

    m.endCycle( command_batch );
}

/*-------------------------------------------------------------------*/
void
DenoiseLogger::addActionLabel( const std::string & label )
{
    Impl & m = *M_impl;
    if ( ! m.opened_ || m.closed_ || m.failed_ || ! m.in_cycle_ || label.empty() ) return;

    m.labels_.push_back( label );
}

/*-------------------------------------------------------------------*/
void
DenoiseLogger::close()
{
    Impl & m = *M_impl;
    if ( m.closed_ ) return; // idempotent

    if ( m.opened_ && ! m.failed_ )
    {
        // a cycle which was begun but not finished is written without commands
        if ( m.in_cycle_ )
        {
            m.endCycle( std::string() );
        }
        m.flushHear( m.last_time_, true );

        // flush and close the streams (the gz stream calls gzclose). meta.json is written
        // after that, so that final = true means that all the tables were closed cleanly.
        if ( ! m.failed_ )
        {
            if ( ! m.closeAll() )
            {
                m.fail( "failed to flush or close a table" );
            }
            else if ( ! m.writeMeta( true ) )
            {
                m.fail( "failed to write meta.json" );
            }
        }
    }

    m.closeAll();
    m.closed_ = true;
}

} // end namespace
