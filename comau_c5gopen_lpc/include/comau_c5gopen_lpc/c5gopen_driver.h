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
#include <map>
#include <string>
#include <vector>
#include <sched.h>
#include <stdlib.h>
#include <thread>

#include <eORL.h>

#include <cnr_logger/cnr_logger.h>
#include <cnr_logger/cnr_logger_macros.h>

#include <comau_c5gopen_lpc/c5gopen_utilities.h>
#include <comau_c5gopen_lpc/realtime_utilities.h>
#include <comau_c5gopen_lpc/realtime_buffer_utils.h>

#define LAST_MESS 0

#define target_pos_max_buff_len 100
#define target_pos_max_buff_log_len 300
namespace c5gopen
{   
  struct RobotJointState
  {
    ORL_joint_value real_pos;
    ORL_joint_value real_vel;
    ORL_joint_value target_pos;
    ORL_joint_value target_vel;
  }__attribute__ ( ( packed ) ); 

  struct RobotCartState
  {
    ORL_cartesian_position real_pos;
    ORL_cartesian_position target_pos;
  }__attribute__ ( ( packed ) ); 

  enum thread_status
  {
    BEFORE_RUN  = 0,
    RUNNING     = 1,
    CLOSED      = 2 
  }__attribute__ ( ( packed ) );

  class C5GOpenDriver: public std::enable_shared_from_this<c5gopen::C5GOpenDriver>
  {
  public:
    typedef std::shared_ptr<C5GOpenDriver> Ptr;
    C5GOpenDriver(const c5gopen::C5GOpenDriverCfg& c5gopen_cfg, 
                  std::shared_ptr<cnr_logger::TraceLogger>& logger ); 

    ~C5GOpenDriver();

    bool init();
    bool run();
    
    thread_status getC5GOpenThreadsStatus();
    thread_status getComThreadsStatus();
    thread_status getLoopConsoleThreadsStatus();

    RobotJointState getRobotJointStateLink( const size_t& arm );
    RobotJointState getRobotJointStateMotor( const size_t& arm );
    RobotCartState getRobotCartState( const size_t& arm );
    ORL_joint_value getRobotMotorCurrent( const size_t& arm );

    bool setRobotJointAbsoluteTargetPosition( const size_t& arm, const RobotJointState& joint_state );

    friend int c5gopen_callback( int input );

    typedef int (C5GOpenDriver::*CF)(int);
    
  private:
  
    // C5GOpen Class configuration parameters
    bool system_initialized_;

    // C5GOpen configuration parameters
    size_t c5gopen_ctrl_idx_orl_; // orl format
    std::string c5gopen_ip_ctrl_;
    std::string c5gopen_sys_id_;

    std::vector<size_t> active_arms_;

    size_t c5gopen_period_orl_; // orl format

    ORL_cartesian_position base_frame_;
    ORL_cartesian_position user_frame_;
    ORL_cartesian_position tool_frame_;

    // C5GOpen thread managment
    std::mutex mtx_;

    std::thread c5gopen_thread_; 
    std::thread com_thread_; 
    std::thread loop_console_thread_;
    std::shared_ptr<cnr_logger::TraceLogger> logger_;

    thread_status c5gopen_threads_status_ = thread_status::BEFORE_RUN;
    thread_status com_threads_status_ = thread_status::BEFORE_RUN;
    thread_status loop_console_threads_status_ = thread_status::BEFORE_RUN;

    // C5GOpen internal control flags 
    std::map<size_t,bool> first_arm_driveon_;
    std::map<size_t,bool> flag_RunningMove_; // to be verified if necessary
    std::map<size_t,bool> flag_ExitFromOpen_;
    std::map<size_t,bool> trajectory_in_execution_; // To be verified if it is necessary
    std::map<size_t,bool> robot_movement_enabled_;
    std::map<size_t,size_t> modality_active_;
    std::map<size_t,size_t> modality_old_;

    std::map<size_t,ORL_joint_value> actual_joints_position_;
    std::map<size_t,ORL_joint_value> starting_jnt_position_;
    std::map<size_t,ORL_joint_value> last_jnt_target_rcv_;
    std::map<size_t,ORL_cartesian_position> actual_cartesian_position_;

    std::map<size_t,realtime_buffer::CircBufferUnqPtr<RobotJointState>> absolute_target_jnt_position_;
 
    // Data structure for logging
    std::map<size_t,RobotJointState> robot_joint_state_link_log_;
    std::map<size_t,RobotJointState> robot_joint_state_motor_log_;
    std::map<size_t,RobotCartState> robot_cart_state_link_log_;
    std::map<size_t,ORL_joint_value> robot_motor_current_log_;

    
    // C5GOpen class internal methods
    void c5gopen_thread( );
    void logging_thread( );
    void loop_console_thread( );
    bool initialize_control_position( void );
    bool update_robot_state( );
    void set_exit_from_open( const size_t& arm );

  };

  int c5gopen_callback( int input );
  void init_driver_library(C5GOpenDriver* c5gopen_driver);

} // end of namespace c5gopen

#endif
