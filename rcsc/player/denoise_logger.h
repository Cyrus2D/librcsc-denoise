// -*-c++-*-

/*!
  \file denoise_logger.h
  \brief per-agent dataset logger for training observation denoisers. Header File
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

#ifndef RCSC_PLAYER_DENOISE_LOGGER_H
#define RCSC_PLAYER_DENOISE_LOGGER_H

#include <rcsc/game_time.h>

#include <memory>
#include <string>

namespace rcsc {

class PlayerAgent;

/*!
  \class DenoiseLogger
  \brief per-agent dataset logger.

  Every cycle the logger writes the raw sensor input, the librcsc estimates,
  the ground truth (server fullstate) and the commands sent by this agent.
  The logger is read-only with respect to the agent: it never changes the
  world model. It is created by PlayerAgent only when the option
  denoise_log_dir is not empty.

  The column dictionary is documented in
  .github/instructions/denoise-dataset.instructions.md.
*/
class DenoiseLogger {
public:
    //! schema version written to meta.json
    static const int SCHEMA_VERSION;

private:

    struct Impl; //!< pimpl idiom

    std::unique_ptr< Impl > M_impl;

public:

    /*!
      \brief create the logger. no file is opened here.
      \param agent reference to the owner agent
     */
    explicit
    DenoiseLogger( const PlayerAgent & agent );

    /*!
      \brief close all files
     */
    ~DenoiseLogger();

    /*!
      \brief called after a server message was handled by PlayerAgent::parse()
      \param msg raw server message
      \param recv_msec local time (milliseconds since epoch) when the message was received
      \param current agent's current game time after the message was analyzed
     */
    void onReceive( const char * msg,
                    const long long recv_msec,
                    const GameTime & current );

    /*!
      \brief called when a message is sent to the server
      \param msg raw message
      \param is_action_batch true if msg is the command batch composed by the action decision
      \param current agent's current game time
     */
    void onSend( const char * msg,
                 const bool is_action_batch,
                 const GameTime & current );

    /*!
      \brief take the pre-decision snapshot. must be called just after
      WorldModel::updateJustBeforeDecision() and before actionImpl().
      \param current current game time
      \param decision_trigger "think", "see" or "timeout"
     */
    void beginCycle( const GameTime & current,
                     const std::string & decision_trigger );

    /*!
      \brief snapshot the predictions that depend on the queued commands.
      must be called before ActionEffector::makeCommand(), which deletes the
      queued command objects. does nothing if beginCycle() was not called in this cycle.
     */
    void preSend();

    /*!
      \brief finalize the cycle. must be called after the command batch was
      composed and before ActionEffector::clearAllCommands().
      \param command_batch command string sent to the server in this cycle. may be empty.
     */
    void endCycle( const std::string & command_batch );

    /*!
      \brief add a behavior label for the current cycle. labels are joined by '|'
      and cleared at the beginning of the next cycle.
      \param label label string
     */
    void addActionLabel( const std::string & label );

    /*!
      \brief close all files, then write the final meta.json (final = true and
      status = "ok" only if every table was flushed and closed successfully). idempotent.
      The destructor does not touch the owner agent; PlayerAgent calls close()
      explicitly while its members are still alive.
     */
    void close();

    /*!
      \brief get the current local time in milliseconds since epoch
     */
    static
    long long nowMsec();
};

} // end namespace

#endif
