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

/* author Enrico Villagrossi (enrico.villagrossi@stiima.cnr.it) */

#ifndef C5GOPEN_DRIVER_H
#define C5GOPEN_DRIVER_H

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
#define TARGET_POS_MAX_BUFF_LEN 5
#define MAX_JNT_VEL_DEG_S 150 // [deg/s] to be acquired from robot controller
#define SIZE_OF_DOUBLE 8

namespace c5gopen
{   
  class C5GOpenDriver: public std::enable_shared_from_this<c5gopen::C5GOpenDriver>
  {
  public:
    typedef std::shared_ptr<C5GOpenDriver> Ptr;
    C5GOpenDriver(const c5gopen::C5GOpenDriverCfg& c5gopen_cfg, 
                  const std::shared_ptr<cnr_logger::TraceLogger>& logger ); 

    ~C5GOpenDriver();

    bool init();
    bool run();
    
    bool getSystemInitialized();

    thread_status getC5GOpenThreadsStatus();
    thread_status getComThreadsStatus();
    thread_status getLoopConsoleThreadsStatus();

    std::map<size_t,RobotJointState> getRobotJointStateLink( );
    std::map<size_t,RobotJointState> getRobotJointStateMotor( );
    std::map<size_t,RobotCartState> getRobotCartState( );
    std::map<size_t,RobotMotorCurrentState> getRobotMotorCurrent( );
    std::map<size_t,RobotJointStateArray> getRobotJointStateLinkArray( );
    std::map<size_t,RobotJointStateArray> getRobotJointStateMotorArray( );
    std::map<size_t,RobotCartStateArray> getRobotCartStateArray( );
    std::map<size_t,RobotGenericArray> getRobotMotorCurrentArray( );

    bool setRobotJointAbsoluteTargetPosition( const size_t& arm, const RobotJointState& joint_state );

    friend int c5gopenCallback( int input );

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
    std::mutex mtx_log_;
    std::mutex mtx_trj_;
    std::mutex mtx_open_;

    std::thread c5gopen_thread_; 
    std::thread logging_thread_; 
    std::thread loop_console_thread_;
    std::shared_ptr<cnr_logger::TraceLogger> logger_;

    thread_status c5gopen_thread_status_ = thread_status::BEFORE_RUN;
    thread_status com_thread_status_ = thread_status::BEFORE_RUN;
    thread_status loop_console_thread_status_ = thread_status::BEFORE_RUN;

    // C5GOpen internal control flags 
    std::map<size_t,bool> first_arm_driveon_;
    std::map<size_t,bool> flag_ExitFromOpen_;
    std::map<size_t,bool> robot_movement_enabled_;
    std::map<size_t,size_t> modality_active_;
    std::map<size_t,size_t> modality_old_;

    std::map<size_t,ORL_joint_value>        actual_joints_position_;
    std::map<size_t,ORL_cartesian_position> actual_cartesian_position_;
    std::map<size_t,ORL_joint_value>        last_jnt_target_rcv_;

    std::map<size_t,RobotJointState> absolute_target_jnt_position_;
    std::map<size_t,realtime_buffer::CircBufferUnqPtr<RobotJointState>> circ_absolute_target_jnt_position_;
 
    // Data structure for logging
    std::map<size_t,RobotJointState>          robot_joint_state_link_log_;
    std::map<size_t,RobotJointState>          robot_joint_state_motor_log_;
    std::map<size_t,RobotCartState>           robot_cart_state_log_;
    std::map<size_t,RobotMotorCurrentState>   robot_motor_current_log_;

    
    // C5GOpen class internal methods
    void c5gopenThread( );
    void loggingThread( );
    void loopConsoleThread( );
    bool initializeControlPosition( );
    bool microinterpolate( );
    bool updateRobotState( );
    void setExitFromOpen( const size_t& arm );

  };

  int c5gopenCallback( int input );
  void initDriverLibrary(C5GOpenDriver* c5gopen_driver);

} // end of namespace c5gopen

#endif
