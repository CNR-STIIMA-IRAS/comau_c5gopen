/*
 *  Software License Agreement (New BSD License)
 *
 *  Copyright 2020 National Council of Research of Italy (CNR)
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __C5GOPEN_LPC_NODE__
#define __C5GOPEN_LPC_NODE__

#include <string>
#include <vector>

#include <sched.h>
#include <stdlib.h>

#include <thread>
#include <boost/circular_buffer.hpp>

#include <eORL.h>

#include <cnr_logger/cnr_logger.h>
#include <cnr_logger/cnr_logger_macros.h>

#include <comau_c5gopen_lpc/realtime_buffer_utils.h>

#define LAST_MESS 0

#define MAX_NUM_ARMS 1 // to be checked and moved in a cfg file  

#define target_pos_max_buff_len 100
#define target_pos_max_buff_log_len 300
#define delta_target_cart_pos_max_len 10
namespace c5gopen
{   
  struct absolute_target_position_t
  {
    ORL_joint_value target_pos;
    ORL_joint_value target_vel;
  }__attribute__ ( ( packed ) ); 

  struct absolute_real_position_t
  {
    ORL_joint_value real_pos;
    ORL_joint_value real_vel;
  }__attribute__ ( ( packed ) ); 

  enum thread_status
  {
    BEFORE_RUN  = 0,
    RUNNING     = 1,
    CLOSED      = 2 
  };

  class C5GOpenDriver: public std::enable_shared_from_this<c5gopen::C5GOpenDriver>
  {
  public:
    typedef std::shared_ptr<C5GOpenDriver> Ptr;
    C5GOpenDriver(const std::string& ip_ctrl, const std::string& sys_id,
                  const int& c5gopen_period, std::shared_ptr<cnr_logger::TraceLogger>& logger ); 

    ~C5GOpenDriver();

    bool init();
    bool run();
    thread_status getC5GOpenThreadsStatus();
    thread_status getComThreadsStatus();
    thread_status getLoopConsoleThreadsStatus();
    
    friend int c5gopen_callback( int input );

    typedef int (C5GOpenDriver::*CF)(int);
    
  private:

    bool c5gopen_cycle_active_ = false;
    bool system_initialized_ = false;

    thread_status c5gopen_threads_status_ = thread_status::BEFORE_RUN;
    thread_status com_threads_status_ = thread_status::BEFORE_RUN;
    thread_status loop_console_threads_status_ = thread_status::BEFORE_RUN;

    int c5gopen_period_; // in microseconds

    std::string ip_ctrl_;
    std::string sys_id_;

    std::mutex mtx_;

    std::thread c5gopen_thread_; 
    std::thread com_thread_; 
    std::thread loop_console_thread_;
    std::shared_ptr<cnr_logger::TraceLogger> logger_;

    bool first_arm_driveon_[MAX_NUM_ARMS]         = {false};
    bool flag_RunningMove_[MAX_NUM_ARMS]          = {false};
    bool flag_ExitFromOpen_[MAX_NUM_ARMS]         = {false};
    bool trajectory_in_execution_[MAX_NUM_ARMS]   = {false};
    bool robot_movement_enabled_[MAX_NUM_ARMS]    = {false};
    unsigned int modality_active_[MAX_NUM_ARMS]   = {0x0};
    unsigned int modality_old_[MAX_NUM_ARMS]      = {0x0};

    ORL_joint_value actual_joints_position_[MAX_NUM_ARMS];
    ORL_cartesian_position actual_cartesian_position_[MAX_NUM_ARMS];

    absolute_target_position_t starting_absolute_jnt_position_[MAX_NUM_ARMS];
    absolute_target_position_t last_absolute_target_jnt_position_rcv_[MAX_NUM_ARMS];


    realtime_buffer::circ_buffer<absolute_target_position_t> absolute_target_jnt_position_[MAX_NUM_ARMS];
    realtime_buffer::circ_buffer<absolute_target_position_t> absolute_target_jnt_position_log_[MAX_NUM_ARMS];

    void c5gopen_thread( );
    void com_thread( );
    void loop_console_thread( );
    int  initialize_control_position( void );
    void set_exit_from_open( const int& iArm );

  };

  int c5gopen_callback( int input );
  void init_driver_library(C5GOpenDriver* c5gopen_driver);

} // end of namespace c5gopen

#endif
