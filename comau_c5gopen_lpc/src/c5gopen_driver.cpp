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

#if PREEMPTIVE_RT
  #include <cinttypes>
  #include <csignal>
  #define USE_TIMER_REALTIME_UTILS  1
  #define PRE_ALLOCATION_SIZE       1024*1024*1024
  #define RT_STACK_SIZE             1024*1024
#else
  #define USE_WALLRATE
#endif

#include <comau_c5gopen_lpc/c5gopen_driver.h>

namespace c5gopen
{
  C5GOpenDriver* g_driver;
   
  void initDriverLibrary(C5GOpenDriver* c5gopen_driver)
  {
    g_driver = c5gopen_driver;
  }

  C5GOpenDriver::C5GOpenDriver( const c5gopen::C5GOpenDriverCfg& c5gopen_cfg, 
                                const std::shared_ptr<cnr_logger::TraceLogger>& logger ): 
                                c5gopen_ctrl_idx_orl_(c5gopen_cfg.ctrl_idx_orl_),  
                                c5gopen_ip_ctrl_(c5gopen_cfg.ip_ctrl_), 
                                c5gopen_sys_id_(c5gopen_cfg.sys_id_), 
                                c5gopen_period_orl_(c5gopen_cfg.c5gopen_period_orl_), 
                                active_arms_(c5gopen_cfg.active_arms_), 
                                base_frame_(c5gopen_cfg.base_frame_),
                                user_frame_(c5gopen_cfg.user_frame_),
                                tool_frame_(c5gopen_cfg.tool_frame_),
                                logger_(logger)
  {
    system_initialized_ = false;

    for ( const size_t& active_arm : active_arms_ )
    {
      // C5GOpen internal control flags initialization
      first_arm_driveon_.insert(std::make_pair(active_arm, false));
      flag_ExitFromOpen_.insert(std::make_pair(active_arm, false));
      robot_movement_enabled_.insert(std::make_pair(active_arm, false));
      first_pos_abs_cycle_.insert(std::make_pair(active_arm, true));
      modality_active_.insert(std::make_pair(active_arm, CRCOPEN_LISTEN));
      modality_old_.insert(std::make_pair(active_arm, CRCOPEN_LISTEN));

      ORL_joint_value joint_value_zero;
      memset(&joint_value_zero,0x0,sizeof(ORL_joint_value));
      actual_joints_position_.insert(std::make_pair(active_arm,joint_value_zero));
      last_jnt_target_rcv_.insert(std::make_pair(active_arm,joint_value_zero));
      starting_abs_pos_.insert(std::make_pair(active_arm,joint_value_zero));
    
      ORL_cartesian_position cart_position_zero;
      memset(&cart_position_zero,0x0,sizeof(ORL_cartesian_position));
      actual_cartesian_position_.insert(std::make_pair(active_arm,cart_position_zero));

      RobotJointState absolute_target_jnt_position_zero;
      memset(&absolute_target_jnt_position_zero,0x0,sizeof(RobotJointState));
      absolute_target_jnt_position_.insert(std::make_pair(active_arm, absolute_target_jnt_position_zero));

      circ_absolute_target_jnt_position_.insert(std::make_pair(active_arm, std::unique_ptr<realtime_buffer::CircBuffer<RobotJointState>>( new realtime_buffer::CircBuffer<RobotJointState>(TARGET_POS_MAX_BUFF_LEN))));

      // Data structure for logging initialization
      RobotJointState joint_state_zero;
      memset(&joint_state_zero,0x0,sizeof(RobotJointState));
      robot_joint_state_link_log_.insert(std::make_pair(active_arm,joint_state_zero));
      robot_joint_state_motor_log_.insert(std::make_pair(active_arm,joint_state_zero));
      
      RobotCartState cart_state_zero;
      memset(&cart_state_zero,0x0,sizeof(RobotCartState));
      robot_cart_state_log_.insert(std::make_pair(active_arm,cart_state_zero));

      RobotMotorCurrentState motor_current_zero;
      memset(&motor_current_zero,0x0,sizeof(RobotMotorCurrentState));
      robot_motor_current_log_.insert(std::make_pair(active_arm,motor_current_zero));
    }

  };

  C5GOpenDriver::~C5GOpenDriver()
  {
    // nothing to be done here
  };

  bool C5GOpenDriver::init( )
  {
    initDriverLibrary(this);
    c5gopen_thread_ = std::thread(&c5gopen::C5GOpenDriver::c5gopenThread, this); 
    logging_thread_ = std::thread(&c5gopen::C5GOpenDriver::loggingThread, this); 
    loop_console_thread_ = std::thread(&c5gopen::C5GOpenDriver::loopConsoleThread, this);

    return true;
  };

  bool C5GOpenDriver::run()
  {
    c5gopen_thread_.detach();
    logging_thread_.detach();
    loop_console_thread_.detach();
    
    return true;
  };

  bool C5GOpenDriver::getSystemInitialized()
  {
    return system_initialized_;
  }

  thread_status C5GOpenDriver::getC5GOpenThreadsStatus()
  {
    return c5gopen_thread_status_;
  }

  thread_status C5GOpenDriver::getComThreadsStatus()
  {
    return com_thread_status_;
  }
    
  thread_status C5GOpenDriver::getLoopConsoleThreadsStatus()
  {
    return loop_console_thread_status_;
  }

  std::map<size_t,RobotJointState> C5GOpenDriver::getRobotJointStateLink( )
  {
    return robot_joint_state_link_log_;
  }

  std::map<size_t,RobotJointState> C5GOpenDriver::getRobotJointStateMotor( )
  {
    return robot_joint_state_motor_log_;
  }

  std::map<size_t,RobotCartState> C5GOpenDriver::getRobotCartState( ) 
  {
    return robot_cart_state_log_;
  }

  std::map<size_t,RobotMotorCurrentState> C5GOpenDriver::getRobotMotorCurrent( )
  {
    return robot_motor_current_log_;
  }

  std::map<size_t,RobotJointStateArray> C5GOpenDriver::getRobotJointStateLinkArray( )
  {
    RobotJointStateArray joint_state_zero;
    memset(&joint_state_zero,0x0,sizeof(RobotJointStateArray));
    std::map<size_t,RobotJointStateArray> robot_joint_state_link_;

    mtx_log_.lock();
    for (const size_t& arm : active_arms_)
    {
      robot_joint_state_link_.insert(std::make_pair(arm,joint_state_zero));
      robot_joint_state_link_[arm].time_us = robot_joint_state_link_log_[arm].time_us;
      memcpy(robot_joint_state_link_[arm].target_pos, robot_joint_state_link_log_[arm].target_pos.value,  sizeof(robot_joint_state_link_log_[arm].target_pos.value)); 
      memcpy(robot_joint_state_link_[arm].real_pos,   robot_joint_state_link_log_[arm].real_pos.value,    sizeof(robot_joint_state_link_log_[arm].real_pos.value)); 
      memcpy(robot_joint_state_link_[arm].target_vel, robot_joint_state_link_log_[arm].target_vel.value,  sizeof(robot_joint_state_link_log_[arm].target_vel.value)); 
      memcpy(robot_joint_state_link_[arm].real_vel,   robot_joint_state_link_log_[arm].real_vel.value,    sizeof(robot_joint_state_link_log_[arm].real_vel.value));
    }
    mtx_log_.unlock();

    return robot_joint_state_link_;
  }

  std::map<size_t,RobotJointStateArray> C5GOpenDriver::getRobotJointStateMotorArray( )
  {
    RobotJointStateArray joint_state_zero;
    memset(&joint_state_zero,0x0,sizeof(RobotJointStateArray));
    std::map<size_t,RobotJointStateArray> robot_joint_state_motor_;
    
    mtx_log_.lock();
    for (const size_t& arm : active_arms_)
    {
      robot_joint_state_motor_.insert(std::make_pair(arm,joint_state_zero));
      robot_joint_state_motor_[arm].time_us = robot_joint_state_motor_log_[arm].time_us;
      memcpy(robot_joint_state_motor_[arm].target_pos, robot_joint_state_motor_log_[arm].target_pos.value,  sizeof(robot_joint_state_motor_log_[arm].target_pos.value)); 
      memcpy(robot_joint_state_motor_[arm].real_pos,   robot_joint_state_motor_log_[arm].real_pos.value,    sizeof(robot_joint_state_motor_log_[arm].real_pos.value)); 
      memcpy(robot_joint_state_motor_[arm].target_vel, robot_joint_state_motor_log_[arm].target_vel.value,  sizeof(robot_joint_state_motor_log_[arm].target_vel.value)); 
      memcpy(robot_joint_state_motor_[arm].real_vel,   robot_joint_state_motor_log_[arm].real_vel.value,    sizeof(robot_joint_state_motor_log_[arm].real_vel.value)); 
    }
    mtx_log_.unlock();

    return robot_joint_state_motor_;
  }

  std::map<size_t,RobotCartStateArray> C5GOpenDriver::getRobotCartStateArray( )
  {
    RobotCartStateArray cart_state_zero;
    memset(&cart_state_zero,0x0,sizeof(RobotCartStateArray));
    std::map<size_t,RobotCartStateArray> robot_cart_state_;

    mtx_log_.lock();
    for (const size_t& arm : active_arms_)
    {
      robot_cart_state_.insert(std::make_pair(arm,cart_state_zero));
      memcpy(robot_cart_state_[arm].config_flags_target, robot_cart_state_log_[arm].target_pos.config_flags, sizeof(robot_cart_state_log_[arm].target_pos.config_flags));
      memcpy(robot_cart_state_[arm].config_flags_real, robot_cart_state_log_[arm].real_pos.config_flags, sizeof(robot_cart_state_log_[arm].real_pos.config_flags));
      
      robot_cart_state_[arm].time_us = robot_cart_state_log_[arm].time_us;

      robot_cart_state_[arm].target_pos[0] = robot_cart_state_log_[arm].target_pos.x;
      robot_cart_state_[arm].target_pos[1] = robot_cart_state_log_[arm].target_pos.y;
      robot_cart_state_[arm].target_pos[2] = robot_cart_state_log_[arm].target_pos.z;
      robot_cart_state_[arm].target_pos[3] = robot_cart_state_log_[arm].target_pos.a;
      robot_cart_state_[arm].target_pos[4] = robot_cart_state_log_[arm].target_pos.e;
      robot_cart_state_[arm].target_pos[5] = robot_cart_state_log_[arm].target_pos.r;

      robot_cart_state_[arm].real_pos[0] = robot_cart_state_log_[arm].real_pos.x;
      robot_cart_state_[arm].real_pos[1] = robot_cart_state_log_[arm].real_pos.y;
      robot_cart_state_[arm].real_pos[2] = robot_cart_state_log_[arm].real_pos.z;
      robot_cart_state_[arm].real_pos[3] = robot_cart_state_log_[arm].real_pos.a;
      robot_cart_state_[arm].real_pos[4] = robot_cart_state_log_[arm].real_pos.e;
      robot_cart_state_[arm].real_pos[5] = robot_cart_state_log_[arm].real_pos.r;
    }
    mtx_log_.unlock();

    return robot_cart_state_;
  }

  std::map<size_t,RobotGenericArray> C5GOpenDriver::getRobotMotorCurrentArray( )
  {
    RobotGenericArray gen_array_zero;
    memset(&gen_array_zero,0x0,sizeof(RobotGenericArray));
    std::map<size_t,RobotGenericArray> robot_motor_current_;

    mtx_log_.lock();
    for (const size_t& arm : active_arms_)
    {
      robot_motor_current_[arm].time_us = robot_motor_current_log_[arm].time_us;
      robot_motor_current_.insert(std::make_pair(arm,gen_array_zero));
      memcpy(robot_motor_current_[arm].value, robot_motor_current_log_[arm].motor_currents.value, sizeof(robot_motor_current_log_[arm].motor_currents.value));
    }
    mtx_log_.unlock();

    return robot_motor_current_;
  }

  bool C5GOpenDriver::setRobotJointAbsoluteTargetPosition( const size_t& arm, const RobotJointState& joint_state )
  {
    // arm starts from 1 to 32

    if (std::find(active_arms_.begin(), active_arms_.end(), arm) == active_arms_.end() )
    {
      CNR_WARN(*logger_, "The joint target position setpoint is defined for arm: " << arm+1
                          << " but the arm is not activated, the desired trajectory will not by applied. Please activate the arm.");
      return false;
    }       

    double max_delta_jnt_pos_deg = MAX_JNT_VEL_DEG_S * get_c5gopen_period_in_usec(c5gopen_period_orl_) * pow(10,-6);

    ORL_joint_value last_jnt_pos;

    if ( g_driver->robot_movement_enabled_[arm] && !g_driver->circ_absolute_target_jnt_position_[arm]->empty() )
    {
      // Robot movement was already enabled so the delta is computed w.r.t 
      // the last point provided to the controller
      last_jnt_pos = last_jnt_target_rcv_[arm];      
    }
    else
    {
      //Robot movement was not enabled yet or the robot moved to the last point received from MQTT
      //the actual joints position is used to compute the delta_jnt_position
      last_jnt_pos = actual_joints_position_[arm];
    }

    if( ORL_joints_conversion(&last_jnt_pos, ORL_POSITION_LINK_DEGREE, ORL_SILENT, g_driver->c5gopen_ctrl_idx_orl_, get_orl_arm_num(arm)) < ORLOPEN_RES_OK )
    {
      ORL_joints_conversion(&last_jnt_pos, ORL_POSITION_LINK_DEGREE, ORL_VERBOSE, g_driver->c5gopen_ctrl_idx_orl_, get_orl_arm_num(arm));
      return false;
    }

    for ( size_t idx=0; idx<sizeof(joint_state.target_pos.value)/SIZE_OF_DOUBLE; idx++ )
    {
      if ( std::isnan(joint_state.target_pos.value[idx]) || std::isinf(joint_state.target_pos.value[idx]) ||
           std::isnan(joint_state.target_vel.value[idx]) || std::isinf(joint_state.target_vel.value[idx]) )  
      {
        CNR_WARN(*logger_, "The trajectory received contains NaN or Inf values. Can't insert the last position/velocity received." );
        return false;
      }

      double delta_jnt_position = std::fabs( joint_state.target_pos.value[idx] - last_jnt_pos.value[idx] );

      if ( delta_jnt_position > max_delta_jnt_pos_deg)
      {
        CNR_WARN( *logger_, "Arm " << arm << " Joint: " << idx+1 << " delta joint position = "<< delta_jnt_position <<  " > max delta joint position = " <<  max_delta_jnt_pos_deg << ".   The difference between the last setpoint received and the previous one exceed from the maximum allowed value." );
        CNR_WARN( *logger_, "Arm " << arm << " Joint: " << idx+1 << " received from MQTT: " << joint_state.target_pos.value[idx] << "  last known joint position: " << last_jnt_pos.value[idx] );
        return false;
      }

    } 
      
    mtx_trj_.lock();
    absolute_target_jnt_position_[arm] = joint_state;       
    mtx_trj_.unlock();

    if ( !g_driver->robot_movement_enabled_[arm])
    {
      // Robot movement is enabled only the first time a new valid target position is received.
      // Robot movement is disabled when the C5GOPEN mode change
      mtx_trj_.lock();
      g_driver->robot_movement_enabled_[arm] = true;
      mtx_trj_.unlock();
      CNR_INFO( *logger_, "Robot movements enabled." );
    }
      
    return true;
  }

  // C5G Open thread
  void C5GOpenDriver::c5gopenThread()
  {
#if PREEMPTIVE_RT
    realtime_utilities::period_info  pinfo;
    if(!realtime_utilities::rt_init_thread(RT_STACK_SIZE, sched_get_priority_max(SCHED_OTHER), SCHED_OTHER, &pinfo, (long)get_c5gopen_period_in_nsec(c5gopen_period_orl_)) )
    {
      CNR_ERROR( *logger_, "Failed in setting thread rt properties. Exit. ");
      std::raise(SIGINT);
      return;
    }
    else
    {
      CNR_INFO( *logger_, "Set realtime scheduling properties.");
    }
#endif

    c5gopen_thread_status_ = thread_status::RUNNING;

    CNR_INFO( *logger_, "Starting C5Gopen thread... " );
    
    CNR_INFO( *logger_, "Initializing controller IP: " << c5gopen_ip_ctrl_ << "  system ID: " << c5gopen_sys_id_ << " controller index: " << c5gopen_ctrl_idx_orl_);

    if( ORLOPEN_initialize_controller ( c5gopen_ip_ctrl_.c_str(), c5gopen_sys_id_.c_str(), ORL_SILENT, c5gopen_ctrl_idx_orl_ ) != ORLOPEN_RES_OK )
    {
      ORLOPEN_initialize_controller ( c5gopen_ip_ctrl_.c_str(), c5gopen_sys_id_.c_str(), ORL_VERBOSE, c5gopen_ctrl_idx_orl_ );
      CNR_ERROR( *logger_, "Error in ORL_initialize_controller " );
      return;
    }
    else 
      CNR_INFO( *logger_, std::string(c5gopen_ip_ctrl_) + ": " + std::string(c5gopen_sys_id_) + ".c5g OK" );

    if ( ORLOPEN_set_period( c5gopen_period_orl_, ORL_SILENT, c5gopen_ctrl_idx_orl_ ) != ORLOPEN_RES_OK )
    {
      ORLOPEN_set_period( c5gopen_period_orl_, ORL_VERBOSE, c5gopen_ctrl_idx_orl_ );
      CNR_ERROR( *logger_, "Error in ORLOPEN_set_period" );
      return;
    }
    else
      CNR_INFO( *logger_, "C5Gopen set period (cycle working frequency): " << c5gopen_period_orl_ );

    for ( const size_t& active_arm : active_arms_)
    {
      if ( ORL_initialize_frames( base_frame_, tool_frame_, user_frame_, ORL_SILENT, c5gopen_ctrl_idx_orl_, get_orl_arm_num( active_arm ) ) != ORLOPEN_RES_OK )  
      {
        ORL_initialize_frames( base_frame_, tool_frame_, user_frame_, ORL_VERBOSE, c5gopen_ctrl_idx_orl_, get_orl_arm_num( active_arm ) );
        CNR_ERROR(  *logger_, "Error in ORL_initialize_frames." );
        return;
      }
    }    
     

    if ( ORLOPEN_SetCallBackFunction( c5gopenCallback, ORL_SILENT, c5gopen_ctrl_idx_orl_ ) < ORLOPEN_RES_OK )
    {
      ORLOPEN_SetCallBackFunction( c5gopenCallback, ORL_VERBOSE, c5gopen_ctrl_idx_orl_ );
      CNR_ERROR( *logger_, "Error in ORLOPEN_SetCallBackFunction.");
      return;
    }
    else
      CNR_INFO( *logger_, "User callback function initialized.");  
    
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    /******************** Start with C5Gopen control *******************/

    if ( ORLOPEN_StartCommunication( ORL_SILENT ) != ORLOPEN_RES_OK )
    {
      ORLOPEN_StartCommunication( ORL_VERBOSE );
      ORLOPEN_GetPowerlinkState( ORL_VERBOSE );
      return;
    }
    else
      CNR_INFO( *logger_, "C5Gopen communication initialized. " );
    
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        
    if ( !initializeControlPosition() ) 
    {
      CNR_ERROR( *logger_, "Can't initialize control position." );
      return;
    }

    CNR_INFO( *logger_, "C5Gopen thread started, entering in the infinite loop." );

    system_initialized_ = true;

    // Enter in the infinite loop
    while ( !std::all_of( flag_ExitFromOpen_.begin(), flag_ExitFromOpen_.end(), [](const std::pair<size_t, bool>& flag){ return flag.second; } ) )
    {
      if ( ORLOPEN_GetPowerlinkState(ORL_SILENT) != PWL_ACTIVE )
      {
        for ( const size_t& active_arm : active_arms_ )
          setExitFromOpen( active_arm );
      }
      std::this_thread::sleep_for(std::chrono::microseconds(1));
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    ORLOPEN_StopCommunication( ORL_VERBOSE );
    CNR_INFO(*logger_, "c5gopen communication stopped.");

    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    ORL_terminate_controller( ORL_VERBOSE, c5gopen_ctrl_idx_orl_ );
    CNR_INFO(*logger_, "c5gopen controller terminated.");

    c5gopen_thread_status_ = thread_status::CLOSED;
    CNR_INFO( *logger_, "c5gopen theread closed." );
  }

  // C5G Open thread
  void C5GOpenDriver::loggingThread()
  {
    com_thread_status_ = thread_status::RUNNING;
        
    CNR_INFO( *logger_, "Communication thread started, entering in the infinite loop." );

    while ( !std::all_of( flag_ExitFromOpen_.begin(), flag_ExitFromOpen_.end(), [](const std::pair<size_t, bool>& flag){ return flag.second; } ) )
    {
      if ( !updateRobotState() )
      {
        CNR_WARN( *logger_, "Error while updating the robot state." );
      }  
      std::this_thread::sleep_for(std::chrono::microseconds((int64_t) c5gopen::get_c5gopen_period_in_usec(c5gopen_period_orl_)));
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    com_thread_status_ = thread_status::CLOSED;
    CNR_INFO( *logger_, "Communication theread closed." );
  }

  // Console thread
  void C5GOpenDriver::loopConsoleThread()
  {
    loop_console_thread_status_ = thread_status::RUNNING;
 
    CNR_INFO( *logger_, "Console thread started, entering in the infinite loop." );

    while ( !std::all_of( flag_ExitFromOpen_.begin(), flag_ExitFromOpen_.end(), [](const std::pair<size_t, bool>& flag){ return flag.second; } ) )
    {
      CNR_INFO( *logger_, "Press 'x' to exit from open" );  
      std::string gl;
      getline(std::cin,gl);
      
      if ( gl.compare("x") == 0 )
      {
        CNR_INFO( *logger_, "Which arm do you want deactivate?[1/2/3/4/All]" );  
        std::string gl;
        getline(std::cin,gl);
        
        if ( gl.compare("All") == 0 )
        {
          for ( const size_t& active_arm : active_arms_ )
            setExitFromOpen( active_arm );

          CNR_INFO( *logger_, "... preparing to exit from c5gopen wait please... " ); 
        }
        else if ( gl.compare("1") == 0 && std::stoul(gl) <= active_arms_.size() )
        {
          setExitFromOpen( ORL_ARM1 );
          CNR_INFO( *logger_, " ... preparing to exit from ARM 1 open modelity wait please... " );
        }
        else if ( gl.compare("2") == 0 && std::stoul(gl) <= active_arms_.size() )
        {
          setExitFromOpen( ORL_ARM2 );
          CNR_INFO( *logger_, " ... preparing to exit from ARM 2 open modelity wait please... " );
        }
        else if ( gl.compare("3") == 0 && std::stoul(gl) <= active_arms_.size() )
        {
          setExitFromOpen( ORL_ARM3 );
          CNR_INFO( *logger_, " ... preparing to exit from ARM 3 open modelity wait please... " );
        }
        else if ( gl.compare("4") == 0 && std::stoul(gl) <= active_arms_.size() )
        {
          setExitFromOpen( ORL_ARM4 );
          CNR_INFO( *logger_, " ... preparing to exit from ARM 4 open modelity wait please... " );
        }
        else
          CNR_INFO( *logger_, " you selected a wrong number of ARM " );
      }
      
      std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    loop_console_thread_status_ = thread_status::CLOSED;

    CNR_INFO( *logger_, "Console theread closed." );
  }

  bool C5GOpenDriver::microinterpolate( )
  {
    // TO BE FINISHED
    return true;
  }

  bool C5GOpenDriver::initializeControlPosition ( )
  {
    size_t modality;
    long output_jntmask;
    std::string s_modality;
    ORL_System_Variable orl_sys_var;

    if ( ORLOPEN_GetPowerlinkState(ORL_VERBOSE) == PWL_ACTIVE )
    {
      for ( const size_t& active_arm : active_arms_ )
      {
        modality        = ORLOPEN_GetModeMasterAx( ORL_SILENT, c5gopen_ctrl_idx_orl_, get_orl_arm_num(active_arm) );
        output_jntmask  = ORLOPEN_GetOpenMask( ORL_SILENT, c5gopen_ctrl_idx_orl_, get_orl_arm_num(active_arm) );
        
        if( !decode_modality( modality, s_modality ) )
        {
          CNR_ERROR( *logger_, "Can't decode C5GOpen modality.");
          return false;
        }
        
        CNR_INFO (*logger_, "\n------ ARM " << active_arm << " MODE " << modality << " " << s_modality << " - " 
                              << ( ( ORLOPEN_GetStatusMasterAx(ORL_SILENT, c5gopen_ctrl_idx_orl_, get_orl_arm_num(active_arm)) == CRCOPEN_STS_DRIVEON ) ? "DRIVE_ON" : "DRIVEOFF")
                              << " mask " << (unsigned int)output_jntmask );              
                
        if ( ORLOPEN_GetStatusMasterAx(ORL_SILENT, c5gopen_ctrl_idx_orl_, get_orl_arm_num(active_arm)) != CRCOPEN_STS_DRIVEON )
          CNR_WARN( *logger_, "ATTENTION: The system is in DRIVEOFF status, first syncronized position can't be reliable, don't forget the DRIVEON! ");
        
        if (first_arm_driveon_[active_arm])
        {
          ORLOPEN_sync_position(&actual_joints_position_[active_arm], ORL_SILENT, c5gopen_ctrl_idx_orl_, get_orl_arm_num(active_arm));
        }
        
        ORL_direct_kinematics(&actual_cartesian_position_[active_arm],&actual_joints_position_[active_arm], ORL_SILENT, c5gopen_ctrl_idx_orl_,get_orl_arm_num(active_arm));
        
        CNR_INFO( *logger_, "ORLOPEN_sync_position Joint " 
                            << actual_joints_position_[active_arm].value[ORL_AX1] << " " 
                            << actual_joints_position_[active_arm].value[ORL_AX2] << " "
                            << actual_joints_position_[active_arm].value[ORL_AX3] << " "
                            << actual_joints_position_[active_arm].value[ORL_AX4] << " "
                            << actual_joints_position_[active_arm].value[ORL_AX5] << " "
                            << actual_joints_position_[active_arm].value[ORL_AX6] << " "
                            << actual_joints_position_[active_arm].value[ORL_AX7] << " "
                            << actual_joints_position_[active_arm].value[ORL_AX8] << " "
                            << actual_joints_position_[active_arm].value[ORL_AX9] << " "
                            << actual_joints_position_[active_arm].value[ORL_AX10] << " Unit type: "
                            << (int16_t)actual_joints_position_[active_arm].unit_type );
        
        CNR_INFO( *logger_, " Pose " 
                            << actual_cartesian_position_[active_arm].x << " "
                            << actual_cartesian_position_[active_arm].y << " "
                            << actual_cartesian_position_[active_arm].z << " "
                            << actual_cartesian_position_[active_arm].a << " "
                            << actual_cartesian_position_[active_arm].e << " "
                            << actual_cartesian_position_[active_arm].r << " "
                            << actual_cartesian_position_[active_arm].config_flags << " Unit type: "
                            << actual_cartesian_position_[active_arm].unit_type );
        
        sprintf((char *)orl_sys_var.sysvar_name,"$ARM_DATA[%d].ARM_OVR",active_arm);
        
        orl_sys_var.ctype = ORL_INT;
        orl_sys_var.iv = 20;
        ORL_set_data( orl_sys_var, ORL_SILENT, c5gopen_ctrl_idx_orl_ );
      }
    }
    else
      return false;
    
    CNR_INFO( *logger_, "Control position initialized. "); 

    return true;
  }

  bool C5GOpenDriver::updateRobotState(  )
  { 
    for ( const size_t& active_arm : active_arms_ ) 
    {
      long mask = ORLOPEN_GetOpenMask ( ORL_SILENT, c5gopen_ctrl_idx_orl_, get_orl_arm_num(active_arm) );
        
      if ( system_initialized_ && first_arm_driveon_[active_arm] )
      {
        mtx_log_.lock();
        // *************************************************************************************
        // Robot State joint position REAL    
        // *************************************************************************************
        int64_t current_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
       
        robot_joint_state_link_log_[active_arm].time_us = current_time;
        robot_cart_state_log_[active_arm].time_us = current_time;
        if ( ORLOPEN_get_pos_measured(&robot_joint_state_link_log_[active_arm].real_pos,&robot_cart_state_log_[active_arm].real_pos, LAST_MESS, ORL_SILENT, g_driver->c5gopen_ctrl_idx_orl_, get_orl_arm_num(active_arm) ) < ORLOPEN_RES_OK )
        {
          ORLOPEN_get_pos_measured(&robot_joint_state_link_log_[active_arm].real_pos,&robot_cart_state_log_[active_arm].real_pos, LAST_MESS, ORL_VERBOSE, g_driver->c5gopen_ctrl_idx_orl_, get_orl_arm_num(active_arm) );
          CNR_ERROR( *logger_, "Error in ORLOPEN_get_pos_measured.");
          return false;
        }

        robot_joint_state_motor_log_[active_arm].time_us = current_time;       
        if ( ORLOPEN_get_pos_measured_mr(&robot_joint_state_motor_log_[active_arm].real_pos, &mask, LAST_MESS, ORL_SILENT, c5gopen_ctrl_idx_orl_, get_orl_arm_num(active_arm) ) < ORLOPEN_RES_OK )
        {
          ORLOPEN_get_pos_measured_mr(&robot_joint_state_motor_log_[active_arm].real_pos, &mask, LAST_MESS, ORL_VERBOSE, c5gopen_ctrl_idx_orl_, get_orl_arm_num(active_arm) );
          CNR_ERROR( *logger_, "Error in ORLOPEN_get_pos_measured_mr.");
          return false;
        }

        // *************************************************************************************
        // Robot State joint velocity REAL    
        // *************************************************************************************
        robot_joint_state_motor_log_[active_arm].time_us = current_time;
        // if ( ORLOPEN_get_speed_measured_mr_per_step(&robot_joint_state_motor_log_[active_arm].real_vel, &mask, LAST_MESS, ORL_SILENT, c5gopen_ctrl_idx_orl_, get_orl_arm_num(active_arm) ) < ORLOPEN_RES_OK )
        // {
        //   ORLOPEN_get_speed_measured_mr_per_step(&robot_joint_state_motor_log_[active_arm].real_vel, &mask, LAST_MESS, ORL_VERBOSE, c5gopen_ctrl_idx_orl_, get_orl_arm_num(active_arm) );
        //   CNR_ERROR( *logger_, "Error in ORLOPEN_get_speed_measured_mr_per_step.");
        //   return false;
        // }

        if ( ORLOPEN_get_speed_measured_mrpm(&robot_joint_state_motor_log_[active_arm].real_vel, &mask, LAST_MESS, ORL_SILENT, c5gopen_ctrl_idx_orl_, get_orl_arm_num(active_arm) ) < ORLOPEN_RES_OK )
        {
          ORLOPEN_get_speed_measured_mrpm(&robot_joint_state_motor_log_[active_arm].real_vel, &mask, LAST_MESS, ORL_VERBOSE, c5gopen_ctrl_idx_orl_, get_orl_arm_num(active_arm) );
          CNR_ERROR( *logger_, "Error in ORLOPEN_get_speed_measured_mrpm.");
          return false;
        }
        
        // *************************************************************************************
        // Robot State joint position TARGET    
        // *************************************************************************************
        robot_joint_state_link_log_[active_arm].time_us = current_time;
        if ( ORLOPEN_get_pos_target( &robot_joint_state_link_log_[active_arm].target_pos,&robot_cart_state_log_[active_arm].target_pos, LAST_MESS, ORL_SILENT, c5gopen_ctrl_idx_orl_, get_orl_arm_num(active_arm) ) < ORLOPEN_RES_OK )
        {
          ORLOPEN_get_pos_target( &robot_joint_state_link_log_[active_arm].target_pos,&robot_cart_state_log_[active_arm].target_pos, LAST_MESS, ORL_VERBOSE, c5gopen_ctrl_idx_orl_, get_orl_arm_num(active_arm) );
          CNR_ERROR( *logger_, "Error in ORLOPEN_get_pos_target.");
          return false;
        }
        
        robot_joint_state_motor_log_[active_arm].time_us = current_time;
        if ( ORLOPEN_get_pos_target_mr( &robot_joint_state_motor_log_[active_arm].target_pos, &mask, LAST_MESS, ORL_SILENT, c5gopen_ctrl_idx_orl_, get_orl_arm_num(active_arm) ) < ORLOPEN_RES_OK )
        {
          ORLOPEN_get_pos_target_mr( &robot_joint_state_motor_log_[active_arm].target_pos, &mask, LAST_MESS, ORL_VERBOSE, c5gopen_ctrl_idx_orl_, get_orl_arm_num(active_arm) );
          CNR_ERROR( *logger_, "Error in ORLOPEN_get_pos_target_mr.");
          return false;
        }
        

        // *************************************************************************************
        // Robot State joint velocity TARGET    
        // *************************************************************************************
        // robot_joint_state_motor_log_[active_arm].time_us = current_time;
        // if ( ORLOPEN_get_speed_target_mr_per_step( &robot_joint_state_motor_log_[active_arm].real_vel, &mask, LAST_MESS, ORL_SILENT, c5gopen_ctrl_idx_orl_, get_orl_arm_num(active_arm) ) < ORLOPEN_RES_OK )
        // {
        //   ORLOPEN_get_speed_target_mr_per_step( &robot_joint_state_motor_log_[active_arm].real_vel, &mask, LAST_MESS, ORL_VERBOSE, c5gopen_ctrl_idx_orl_, get_orl_arm_num(active_arm) );
        //   CNR_ERROR( *logger_, "Error in ORLOPEN_get_speed_target_mr_per_step.");
        //   return NULL;
        // }

        if ( ORLOPEN_get_speed_target_mrpm( &robot_joint_state_motor_log_[active_arm].real_vel, &mask, LAST_MESS, ORL_SILENT, c5gopen_ctrl_idx_orl_, get_orl_arm_num(active_arm) ) < ORLOPEN_RES_OK )
        {
          ORLOPEN_get_speed_target_mrpm( &robot_joint_state_motor_log_[active_arm].real_vel, &mask, LAST_MESS, ORL_VERBOSE, c5gopen_ctrl_idx_orl_, get_orl_arm_num(active_arm) );
          CNR_ERROR( *logger_, "Error in ORLOPEN_get_speed_target_mrpm.");
          return NULL;
        }
        
        
        // *************************************************************************************
        // Robot State joint current REAL    
        // *************************************************************************************
        robot_motor_current_log_[active_arm].time_us = current_time;
        if ( ORLOPEN_get_current_measured (&robot_motor_current_log_[active_arm].motor_currents, &mask, LAST_MESS, ORL_SILENT, c5gopen_ctrl_idx_orl_, get_orl_arm_num(active_arm) ) < ORLOPEN_RES_OK )
        {
          ORLOPEN_get_current_measured (&robot_motor_current_log_[active_arm].motor_currents, &mask, LAST_MESS, ORL_VERBOSE, c5gopen_ctrl_idx_orl_, get_orl_arm_num(active_arm) );
          CNR_ERROR( *logger_, "Error in ORLOPEN_get_current_measured.");
          return NULL;
        }
        mtx_log_.unlock();
      }
    }

    return true;
  }

  void C5GOpenDriver::setExitFromOpen( const size_t& arm )
  {
    mtx_open_.lock();
    flag_ExitFromOpen_[arm]         = true;
    mtx_open_.unlock();
  }
  
  int c5gopenCallback ( int input ) 
  { 
    bool arm_driveon;
    std::string s_modality;
    std::map<size_t,bool> flag_new_modality;
    
    for ( const size_t& active_arm : g_driver->active_arms_ ) 
    {
      if ( !g_driver->flag_ExitFromOpen_[active_arm] )
      {
        flag_new_modality.insert(std::make_pair(active_arm,false));
        g_driver->modality_old_[active_arm] = g_driver->modality_active_[active_arm];
        
        g_driver->modality_active_[active_arm]  = ORLOPEN_GetModeMasterAx( ORL_SILENT, g_driver->c5gopen_ctrl_idx_orl_, get_orl_arm_num(active_arm) );
        
        arm_driveon = ( ORLOPEN_GetStatusMasterAx(ORL_SILENT, g_driver->c5gopen_ctrl_idx_orl_, get_orl_arm_num(active_arm) ) == CRCOPEN_STS_DRIVEON ) ? true : false;
        
        if ( arm_driveon && !g_driver->first_arm_driveon_[active_arm] )
           g_driver->first_arm_driveon_[active_arm] = true;
        
        if ( !decode_modality( (size_t) g_driver->modality_active_[active_arm], s_modality ))
        {
          CNR_WARN( g_driver->logger_, "Can't decode C5GOpen modality: skipping c5gopen callback execution.");
          continue;
        }
            
        if( g_driver->modality_old_[active_arm] != g_driver->modality_active_[active_arm] ) 
        {
          flag_new_modality[active_arm] = true;
          CNR_INFO( g_driver->logger_, "ARM " << active_arm << " Modality" << (size_t)g_driver->modality_active_[active_arm] << " " <<  s_modality );
        }   
        else 
          flag_new_modality[active_arm] = false;
        
        switch ( g_driver->modality_active_[active_arm] ) 
        {
          // LISTEN MODE
          case CRCOPEN_LISTEN:
            if ( g_driver->system_initialized_ )
            { 
              if ( g_driver->first_arm_driveon_[active_arm] )
              {
                if ( ORLOPEN_sync_position( &g_driver->actual_joints_position_[active_arm], ORL_SILENT, g_driver->c5gopen_ctrl_idx_orl_, get_orl_arm_num(active_arm) ) < ORLOPEN_RES_OK )
                {
                  ORLOPEN_sync_position( &g_driver->actual_joints_position_[active_arm], ORL_VERBOSE, g_driver->c5gopen_ctrl_idx_orl_, get_orl_arm_num(active_arm) );
                  CNR_ERROR(  g_driver->logger_, "Error in ORLOPEN_sync_position.");
                  return ORLOPEN_ERROR;
                }
              }
              
              if ( ORLOPEN_get_pos_measured(&g_driver->actual_joints_position_[active_arm],&g_driver->actual_cartesian_position_[active_arm], LAST_MESS, ORL_SILENT, g_driver->c5gopen_ctrl_idx_orl_, get_orl_arm_num(active_arm) ) < ORLOPEN_RES_OK )
              {
                ORLOPEN_get_pos_measured(&g_driver->actual_joints_position_[active_arm],&g_driver->actual_cartesian_position_[active_arm], LAST_MESS, ORL_VERBOSE, g_driver->c5gopen_ctrl_idx_orl_, get_orl_arm_num(active_arm) );
                CNR_ERROR(  g_driver->logger_, "Error in ORLOPEN_get_pos_measured.");
                return ORLOPEN_ERROR;
              }
            }
            
            break;

          // ABSOLUTE MODE     
          case CRCOPEN_POS_ABSOLUTE:
            if ( g_driver->system_initialized_ )
            {
              if ( (flag_new_modality[active_arm]) && (g_driver->modality_active_[active_arm] == CRCOPEN_POS_ABSOLUTE) )
              {
                CNR_INFO(  g_driver->logger_, "Modality CRCOPEN_POS_ABSOLUTE activated. ARM: " << active_arm << " --> " << ( (arm_driveon == true) ? "DRIVEON" : "DRIVEOFF") );
                g_driver->mtx_trj_.lock();
                g_driver->robot_movement_enabled_[active_arm] = false;
                g_driver->first_pos_abs_cycle_[active_arm] = true;
                g_driver->mtx_trj_.unlock();
              }
            }
            
            if ( arm_driveon )
            {
              if ( ORLOPEN_get_pos_measured(&g_driver->actual_joints_position_[active_arm],&g_driver->actual_cartesian_position_[active_arm], LAST_MESS, ORL_SILENT, g_driver->c5gopen_ctrl_idx_orl_, get_orl_arm_num(active_arm) ) < ORLOPEN_RES_OK )
              {
                ORLOPEN_get_pos_measured(&g_driver->actual_joints_position_[active_arm],&g_driver->actual_cartesian_position_[active_arm], LAST_MESS, ORL_VERBOSE, g_driver->c5gopen_ctrl_idx_orl_, get_orl_arm_num(active_arm) );
                CNR_ERROR(  g_driver->logger_, "Error in ORLOPEN_get_pos_measured.");
                return ORLOPEN_ERROR;
              }

              if ( g_driver->robot_movement_enabled_[active_arm] )
              {  
                if ( g_driver->circ_absolute_target_jnt_position_[active_arm]->full() )
                {
                  g_driver->circ_absolute_target_jnt_position_[active_arm]->pop_front();
                  g_driver->circ_absolute_target_jnt_position_[active_arm]->push_back(g_driver->absolute_target_jnt_position_[active_arm]);
                }
                else
                  g_driver->circ_absolute_target_jnt_position_[active_arm]->push_back(g_driver->absolute_target_jnt_position_[active_arm]);


                g_driver->microinterpolate(); // FORESEEN BUT NOT IMPLEMENTED YET
                
                g_driver->mtx_trj_.lock();
                g_driver->last_jnt_target_rcv_[active_arm] = g_driver->circ_absolute_target_jnt_position_[active_arm]->front().target_pos;  
                g_driver->circ_absolute_target_jnt_position_[active_arm]->pop_front();
                g_driver->mtx_trj_.unlock();

                int res = ORLOPEN_set_absolute_pos_target_degree( &g_driver->last_jnt_target_rcv_[active_arm], ORL_SILENT, g_driver->c5gopen_ctrl_idx_orl_, get_orl_arm_num(active_arm) );
                if ( res != ORLOPEN_RES_OK )
                {
                  CNR_ERROR(  g_driver->logger_, "Error in ORLOPEN_set_absolute_pos_target_degree. " << ORL_decode_Error_Code(res) );
                  return ORLOPEN_ERROR;  
                }
                
              }
              else
              {  
                if (g_driver->first_pos_abs_cycle_[active_arm])
                {
                  g_driver->first_pos_abs_cycle_[active_arm] = false;
                  g_driver->starting_abs_pos_[active_arm] = g_driver->actual_joints_position_[active_arm];
                }

                if( ORL_joints_conversion(&g_driver->starting_abs_pos_[active_arm], ORL_POSITION_LINK_DEGREE, ORL_SILENT, g_driver->c5gopen_ctrl_idx_orl_, get_orl_arm_num(active_arm)) < ORLOPEN_RES_OK )
                {
                  ORL_joints_conversion(&g_driver->starting_abs_pos_[active_arm], ORL_POSITION_LINK_DEGREE, ORL_VERBOSE, g_driver->c5gopen_ctrl_idx_orl_, get_orl_arm_num(active_arm)); 
                }

                //int res = ORLOPEN_set_absolute_pos_target_degree( &g_driver->actual_joints_position_[active_arm], ORL_SILENT, g_driver->c5gopen_ctrl_idx_orl_, get_orl_arm_num(active_arm) );
                int res = ORLOPEN_set_absolute_pos_target_degree( &g_driver->starting_abs_pos_[active_arm], ORL_SILENT, g_driver->c5gopen_ctrl_idx_orl_, get_orl_arm_num(active_arm) );

                if ( res != ORLOPEN_RES_OK )
                {
                  CNR_ERROR(  g_driver->logger_, "Error in ORLOPEN_set_absolute_pos_target_degree. " << ORL_decode_Error_Code(res) );
                  return ORLOPEN_ERROR;  
                }
                                  
              }
            }
            
            break;
            
          // ADDITIVE MODE   
          case CRCOPEN_POS_ADDITIVE:
            CNR_WARN( g_driver->logger_, "The CRCOPEN_POS_ADDITIVE mode is not currently implemented.");
            break;

          case CRCOPEN_POS_RELATIVE:
            CNR_WARN( g_driver->logger_, "The CRCOPEN_POS_RELATIVE mode is not currently implemented.");
            break;

          case CRCOPEN_POS_ADDITIVE_SB:
            CNR_WARN( g_driver->logger_, "The CRCOPEN_POS_ADDITIVE_SB mode is not currently implemented.");
            break;

          case CRCOPEN_POS_ADDITIVE_SBE:
            CNR_WARN( g_driver->logger_, "The CRCOPEN_POS_ADDITIVE_SBE mode is not currently implemented.");
            break;
        
        }
      }
      else
      {
        ORLOPEN_ExitFromOpen( ORL_VERBOSE, g_driver->c5gopen_ctrl_idx_orl_, get_orl_arm_num(active_arm) );
        CNR_WARN( g_driver->logger_, "Called function ORLOPEN_ExitFromOpen...");
      }
    }
    
    return ORLOPEN_RES_OK;
  }

} // end of namespace c5gopen