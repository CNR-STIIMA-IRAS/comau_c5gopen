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

  void init_driver_library(C5GOpenDriver* c5gopen_driver)
  {
    g_driver = c5gopen_driver;
  }

  C5GOpenDriver::C5GOpenDriver( const c5gopen::C5GOpenCfg& c5gopen_cfg, 
                                std::shared_ptr<cnr_logger::TraceLogger>& logger): 
                                c5gopen_ip_ctrl_(c5gopen_cfg.ip_ctrl_), 
                                c5gopen_sys_id_(c5gopen_cfg.sys_id_), 
                                c5gopen_period_orl_(c5gopen_cfg.c5gopen_period_orl_), 
                                max_number_of_arms_(c5gopen_cfg.max_number_of_arms_),
                                active_arms_(c5gopen_cfg.active_arms_), 
                                base_frame_(c5gopen_cfg.base_frame_),
                                user_frame_(c5gopen_cfg.user_frame_),
                                tool_frame_(c5gopen_cfg.tool_frame_),
                                mqtt_client_id_(c5gopen_cfg.mqtt_client_id_),
                                mqtt_broker_address_(c5gopen_cfg.mqtt_broker_address_),
                                mqtt_port_(c5gopen_cfg.mqtt_port_),
                                mqtt_topic_(c5gopen_cfg.mqtt_topic_),
                                logger_(logger)
  {
    max_number_of_arms_ = active_arms_.size();  
    system_initialized_ = false;

    for ( const size_t& active_arm : active_arms_ )
    {
      first_arm_driveon_.insert(std::make_pair(active_arm, false));
      flag_RunningMove_.insert(std::make_pair(active_arm, false)); // To be verified if it is necessary
      flag_ExitFromOpen_.insert(std::make_pair(active_arm, false));
      trajectory_in_execution_.insert(std::make_pair(active_arm, false)); // To be verified if it is necessary
      robot_movement_enabled_.insert(std::make_pair(active_arm, false));
      modality_active_.insert(std::make_pair(active_arm, CRCOPEN_LISTEN));
      modality_old_.insert(std::make_pair(active_arm, CRCOPEN_LISTEN));

      ORL_joint_value joint_value;
      memset(&joint_value,0x0,sizeof(ORL_joint_value));
      actual_joints_position_.insert(std::make_pair(active_arm,joint_value));
    
      ORL_cartesian_position cart_position;
      memset(&cart_position,0x0,sizeof(ORL_cartesian_position));
      actual_cartesian_position_.insert(std::make_pair(active_arm,cart_position));

      absolute_target_position_t jnt_target_position;
      memset(&jnt_target_position,0x0,sizeof(absolute_target_position_t));
      
      starting_absolute_jnt_position_.insert(std::make_pair(active_arm,jnt_target_position));
      last_absolute_target_jnt_position_rcv_.insert(std::make_pair(active_arm,jnt_target_position));

      absolute_target_jnt_position_.insert(std::make_pair(active_arm, std::unique_ptr<realtime_buffer::CircBuffer<absolute_target_position_t>>( new realtime_buffer::CircBuffer<absolute_target_position_t>(target_pos_max_buff_len))));
      absolute_target_jnt_position_log_.insert(std::make_pair(active_arm, std::unique_ptr<realtime_buffer::CircBuffer<absolute_target_position_t>>( new realtime_buffer::CircBuffer<absolute_target_position_t>(target_pos_max_buff_log_len))));
    }

  };

  C5GOpenDriver::~C5GOpenDriver()
  {
    // nothing to be done here
  };

  bool C5GOpenDriver::init( )
  {
    init_driver_library(this);
    c5gopen_thread_ = std::thread(&c5gopen::C5GOpenDriver::c5gopen_thread, this); 
    com_thread_ = std::thread(&c5gopen::C5GOpenDriver::com_thread, this); 
    loop_console_thread_ = std::thread(&c5gopen::C5GOpenDriver::loop_console_thread, this);

    // something else to be done here

    return true;
  };

  bool C5GOpenDriver::run()
  {
    c5gopen_thread_.detach();
    com_thread_.detach();
    loop_console_thread_.detach();
    
    // something else to be done here
    
    return true;
  };

  thread_status C5GOpenDriver::getC5GOpenThreadsStatus()
  {
    return c5gopen_threads_status_;
  }

  thread_status C5GOpenDriver::getComThreadsStatus()
  {
    return com_threads_status_;
  }
    
  thread_status C5GOpenDriver::getLoopConsoleThreadsStatus()
  {
    return loop_console_threads_status_;
  }

  // C5G Open thread
  void C5GOpenDriver::c5gopen_thread()
  {
#if PREEMPTIVE_RT
    realtime_utilities::period_info  pinfo;
    if(!realtime_utilities::rt_init_thread(RT_STACK_SIZE, sched_get_priority_max(SCHED_OTHER), SCHED_OTHER, &pinfo, (long)get_c5gopen_period_in_nsec(c5gopen_period_orl_)) )
    {
      CNR_ERROR( *logger_, "Failed in setting thread rt properties. Exit. ");
      std::raise(SIGINT);
      return;
    }
#endif

    c5gopen_threads_status_ = thread_status::RUNNING;

    CNR_INFO( *logger_, "C5Gopen thread started. " );

    CNR_WARN( *logger_, "C5Gopen thread started: " <<  c5gopen_ip_ctrl_  << "  " << c5gopen_sys_id_ << "  " <<  c5gopen_ctrl_idx_ );

    if( ORLOPEN_initialize_controller ( c5gopen_ip_ctrl_.c_str(), c5gopen_sys_id_.c_str(), ORL_SILENT, c5gopen_ctrl_idx_ ) != ORLOPEN_RES_OK )
    {
      ORLOPEN_initialize_controller ( c5gopen_ip_ctrl_.c_str(), c5gopen_sys_id_.c_str(), ORL_VERBOSE, c5gopen_ctrl_idx_ );
      CNR_ERROR( *logger_, "Error in ORL_initialize_controller " );
      return;
    }
    else 
      CNR_INFO( *logger_, std::string(c5gopen_ip_ctrl_) + ": " + std::string(c5gopen_sys_id_) + ".c5g OK" );

    if ( ORLOPEN_set_period( c5gopen_period_orl_, ORL_SILENT, c5gopen_ctrl_idx_ ) != ORLOPEN_RES_OK )
    {
      ORLOPEN_set_period( c5gopen_period_orl_, ORL_VERBOSE, c5gopen_ctrl_idx_ );
      CNR_ERROR( *logger_, "Error in ORLOPEN_set_period" );
      return;
    }
    else
      CNR_INFO( *logger_, "C5Gopen set period (cycle working frequency). " );

    for ( const size_t& active_arm : active_arms_)
    {
      if ( ORL_initialize_frames( base_frame_, tool_frame_, user_frame_, ORL_SILENT, c5gopen_ctrl_idx_, get_orl_arm_num( active_arm ) ) != ORLOPEN_RES_OK )  
      {
        ORL_initialize_frames( base_frame_, tool_frame_, user_frame_, ORL_VERBOSE, c5gopen_ctrl_idx_, get_orl_arm_num( active_arm ) );
        CNR_ERROR(  *logger_, "Error in ORL_initialize_frames." );
        return;
      }
    }    
     

    if ( ORLOPEN_SetCallBackFunction( c5gopen_callback, ORL_SILENT, c5gopen_ctrl_idx_ ) < ORLOPEN_RES_OK )
    {
      ORLOPEN_SetCallBackFunction( c5gopen_callback, ORL_VERBOSE, c5gopen_ctrl_idx_ );
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
        
    if ( !initialize_control_position() ) 
    {
      CNR_ERROR( *logger_, "Can't initialize control position." );
      return;
    }

    CNR_INFO( *logger_, "C5Gopen theread started." );

    system_initialized_ = true;

    // Enter in the infinite loop
    while ( !std::all_of( flag_ExitFromOpen_.begin(), flag_ExitFromOpen_.end(), [](const std::pair<size_t, bool>& flag){ return flag.second; } ) )
    {
      if ( ORLOPEN_GetPowerlinkState(ORL_SILENT) != PWL_ACTIVE )
      {
        for ( const size_t& active_arm : active_arms_ )
          set_exit_from_open( active_arm );
      }
 
      std::this_thread::sleep_for(std::chrono::milliseconds(100)); // need to use varibale to set sleep_for function
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    ORLOPEN_StopCommunication( ORL_VERBOSE );
    CNR_INFO(*logger_, "c5gopen communication stopped.");

    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    ORL_terminate_controller( ORL_VERBOSE, c5gopen_ctrl_idx_ );
    CNR_INFO(*logger_, "c5gopen controller terminated.");

    c5gopen_threads_status_ = thread_status::CLOSED;
    CNR_INFO( *logger_, "c5gopen theread closed." );
  }

  // C5G Open thread
  void C5GOpenDriver::com_thread()
  {
    com_threads_status_ = thread_status::RUNNING;
    
    while ( !std::all_of( flag_ExitFromOpen_.begin(), flag_ExitFromOpen_.end(), [](const std::pair<size_t, bool>& flag){ return flag.second; } ) )
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    com_threads_status_ = thread_status::CLOSED;
    CNR_INFO( *logger_, "Communication theread closed." );
  }

  // Console thread
  void C5GOpenDriver::loop_console_thread()
  {
    loop_console_threads_status_ = thread_status::RUNNING;
 
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
            set_exit_from_open( active_arm );

          CNR_INFO( *logger_, "... preparing to exit from c5gopen wait please... " ); 
        }
        else if ( gl.compare("1") == 0 && std::stoul(gl) <= max_number_of_arms_ )
        {
          set_exit_from_open( ORL_ARM1 );
          CNR_INFO( *logger_, " ... preparing to exit from ARM 1 open modelity wait please... " );
        }
        else if ( gl.compare("2") == 0 && std::stoul(gl) <= max_number_of_arms_ )
        {
          set_exit_from_open( ORL_ARM2 );
          CNR_INFO( *logger_, " ... preparing to exit from ARM 2 open modelity wait please... " );
        }
        else if ( gl.compare("3") == 0 && std::stoul(gl) <= max_number_of_arms_ )
        {
          set_exit_from_open( ORL_ARM3 );
          CNR_INFO( *logger_, " ... preparing to exit from ARM 3 open modelity wait please... " );
        }
        else if ( gl.compare("4") == 0 && std::stoul(gl) <= max_number_of_arms_ )
        {
          set_exit_from_open( ORL_ARM4 );
          CNR_INFO( *logger_, " ... preparing to exit from ARM 4 open modelity wait please... " );
        }
        else
          CNR_INFO( *logger_, " you selected a wrong number of ARM " );
      }
      
      std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    loop_console_threads_status_ = thread_status::CLOSED;

    CNR_INFO( *logger_, "Console theread closed." );
  }

  bool C5GOpenDriver::initialize_control_position ( void )
  {
    size_t modality;
    long output_jntmask;
    std::string s_modality;
    ORL_System_Variable orl_sys_var;

    if ( ORLOPEN_GetPowerlinkState(ORL_VERBOSE) == PWL_ACTIVE )
    {
      for ( const size_t& active_arm : active_arms_ )
      {
        modality        = ORLOPEN_GetModeMasterAx( ORL_SILENT, c5gopen_ctrl_idx_, get_orl_arm_num(active_arm) );
        output_jntmask  = ORLOPEN_GetOpenMask( ORL_SILENT, c5gopen_ctrl_idx_, get_orl_arm_num(active_arm) );
        
        if( !decode_modality( modality, s_modality ) )
        {
          CNR_ERROR( *logger_, "Can't decode C5GOpen modality.");
          return false;
        }
        
        CNR_INFO (*logger_, "\n------ ARM " << active_arm << " MODE " << modality << " " << s_modality << " - " 
                              << ( ( ORLOPEN_GetStatusMasterAx(ORL_SILENT, c5gopen_ctrl_idx_, get_orl_arm_num(active_arm)) == CRCOPEN_STS_DRIVEON ) ? "DRIVE_ON" : "DRIVEOFF")
                              << " mask " << (unsigned int)output_jntmask );              
                
        if ( ORLOPEN_GetStatusMasterAx(ORL_SILENT, c5gopen_ctrl_idx_, get_orl_arm_num(active_arm)) != CRCOPEN_STS_DRIVEON )
          CNR_WARN( *logger_, "ATTENTION: The system is in DRIVEOFF status, first syncronized position can't be reliable, don't forget the DRIVEON! ");
        
        if (first_arm_driveon_[active_arm])
        {
          ORLOPEN_sync_position(&actual_joints_position_[active_arm], ORL_SILENT, c5gopen_ctrl_idx_, get_orl_arm_num(active_arm));
        }
        
        ORL_direct_kinematics(&actual_cartesian_position_[active_arm],&actual_joints_position_[active_arm], ORL_SILENT, c5gopen_ctrl_idx_,get_orl_arm_num(active_arm));
        
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
                            << (int16_t)actual_joints_position_[active_arm].unit_type );
        
        CNR_INFO( *logger_, " Pose " 
                            << actual_cartesian_position_[active_arm].x << " "
                            << actual_cartesian_position_[active_arm].y << " "
                            << actual_cartesian_position_[active_arm].z << " "
                            << actual_cartesian_position_[active_arm].a << " "
                            << actual_cartesian_position_[active_arm].e << " "
                            << actual_cartesian_position_[active_arm].r << " "
                            << actual_cartesian_position_[active_arm].config_flags);
        
        sprintf((char *)orl_sys_var.sysvar_name,"$ARM_DATA[%d].ARM_OVR",active_arm);
        
        orl_sys_var.ctype = ORL_INT;
        orl_sys_var.iv = 20;
        ORL_set_data( orl_sys_var, ORL_SILENT, c5gopen_ctrl_idx_ );
      }
    }
    else
      return false;
    
    return true;
  }

  void C5GOpenDriver::set_exit_from_open( const size_t& arm )
  {
    mtx_.lock();
    flag_ExitFromOpen_[arm]         = true;
    trajectory_in_execution_[arm]   = false; 
    mtx_.unlock();
  }

  int c5gopen_callback ( int input ) 
  { 
    long mask;
    bool arm_driveon;
    std::string s_modality;
    std::map<size_t,bool> flag_new_modality;
    
    for ( const size_t& active_arm : g_driver->active_arms_ ) 
    {
      if ( !g_driver->flag_ExitFromOpen_[active_arm] )
      {
        flag_new_modality.insert(std::pair<size_t,bool>(active_arm,false));
        g_driver->modality_old_[active_arm] = g_driver->modality_active_[active_arm];
        
        mask = ORLOPEN_GetOpenMask ( ORL_SILENT, g_driver->c5gopen_ctrl_idx_, get_orl_arm_num(active_arm) );
        g_driver->modality_active_[active_arm]  = ORLOPEN_GetModeMasterAx( ORL_SILENT, g_driver->c5gopen_ctrl_idx_, get_orl_arm_num(active_arm) );
        
        arm_driveon = ( ORLOPEN_GetStatusMasterAx(ORL_SILENT, g_driver->c5gopen_ctrl_idx_, get_orl_arm_num(active_arm) ) == CRCOPEN_STS_DRIVEON ) ? true : false;
        
        if ( arm_driveon && ! g_driver->first_arm_driveon_[active_arm] )
           g_driver->first_arm_driveon_[active_arm] = true;
        
        if ( !decode_modality( (size_t) g_driver->modality_active_[active_arm], s_modality ))
        {
          CNR_WARN( g_driver->logger_, "Can't decode C5GOpen modality: skipping c5gopen callback execution.");
          continue;
        }
            
        if(  g_driver->modality_old_[active_arm] !=  g_driver->modality_active_[active_arm] ) 
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
                if ( ORLOPEN_sync_position( &g_driver->actual_joints_position_[active_arm], ORL_SILENT, g_driver->c5gopen_ctrl_idx_, get_orl_arm_num(active_arm) ) < ORLOPEN_RES_OK )
                {
                  ORLOPEN_sync_position( &g_driver->actual_joints_position_[active_arm], ORL_VERBOSE, g_driver->c5gopen_ctrl_idx_, get_orl_arm_num(active_arm) );
                  CNR_ERROR(  g_driver->logger_, "Error in ORLOPEN_sync_position.");
                  return ORLOPEN_ERROR;
                }
              }
              
              ORL_joint_value actual_joints_position_tmp [g_driver->max_number_of_arms_];
              ORL_cartesian_position actual_cartesian_position_tmp [g_driver->max_number_of_arms_];
              
              if ( ORLOPEN_get_pos_measured(&actual_joints_position_tmp[active_arm],&actual_cartesian_position_tmp[active_arm], LAST_MESS, ORL_SILENT, g_driver->c5gopen_ctrl_idx_, get_orl_arm_num(active_arm) ) < ORLOPEN_RES_OK )
              {
                ORLOPEN_get_pos_measured(&actual_joints_position_tmp[active_arm],&actual_cartesian_position_tmp[active_arm], LAST_MESS, ORL_VERBOSE, g_driver->c5gopen_ctrl_idx_, get_orl_arm_num(active_arm) );
                CNR_ERROR(  g_driver->logger_, "Error in ORLOPEN_get_pos_measured.");
                return ORLOPEN_ERROR;
              }      
              
              memset( &g_driver->starting_absolute_jnt_position_[active_arm], 0x00, sizeof(absolute_target_position_t) );
              memcpy( &g_driver->starting_absolute_jnt_position_[active_arm], & g_driver->actual_joints_position_[active_arm], sizeof(absolute_target_position_t ));
              
              if (  g_driver->absolute_target_jnt_position_log_[active_arm]->full() )
                g_driver-> absolute_target_jnt_position_log_[active_arm]->pop_front();

              g_driver->absolute_target_jnt_position_log_[active_arm]->push_back( g_driver->starting_absolute_jnt_position_[active_arm] );
                
            }
            
            break;

          // ABSOLUTE MODE     
          case CRCOPEN_POS_ABSOLUTE:
            if ( g_driver->system_initialized_ )
            {
              if ( (flag_new_modality[active_arm]) && (g_driver->modality_active_[active_arm] == CRCOPEN_POS_ABSOLUTE) )
              {
                CNR_INFO(  g_driver->logger_, "Modality CRCOPEN_POS_ABSOLUTE activated." );
                g_driver->robot_movement_enabled_[active_arm] = false;
              }
            }
            
            if ( arm_driveon )
            {
              if ( g_driver->robot_movement_enabled_[active_arm] )
              {  
                if ( !g_driver->absolute_target_jnt_position_[active_arm]->empty() )
                {
                  memcpy( &g_driver->last_absolute_target_jnt_position_rcv_[active_arm], & g_driver->absolute_target_jnt_position_[active_arm]->front(), sizeof(absolute_target_position_t ));
                   g_driver->absolute_target_jnt_position_[active_arm]->pop_front();
                  
                  if ( ORLOPEN_set_absolute_pos_target_degree( &g_driver->last_absolute_target_jnt_position_rcv_[active_arm].target_pos, ORL_SILENT, g_driver->c5gopen_ctrl_idx_, get_orl_arm_num(active_arm) ) != ORLOPEN_RES_OK )
                    CNR_ERROR(  g_driver->logger_, "Error in ORLOPEN_set_absolute_pos_target_degree." );
                  
                  ORL_axis_2_joints( &g_driver->last_absolute_target_jnt_position_rcv_[active_arm].target_pos, ORL_SILENT, g_driver->c5gopen_ctrl_idx_, get_orl_arm_num(active_arm) );

                  g_driver->last_absolute_target_jnt_position_rcv_[active_arm].target_vel.unit_type = ORL_SPEED_LINK_DEGREE_SEC;
                  ORL_joints_conversion( &g_driver->last_absolute_target_jnt_position_rcv_[active_arm].target_vel, ORL_SPEED_MOTORROUNDS, ORL_SILENT, g_driver->c5gopen_ctrl_idx_, get_orl_arm_num(active_arm) );
                  
                  if ( ORLOPEN_set_ExtData( &g_driver->last_absolute_target_jnt_position_rcv_[active_arm].target_vel, &mask, ORL_SILENT, g_driver->c5gopen_ctrl_idx_, get_orl_arm_num(active_arm) ) != ORLOPEN_RES_OK )
                    CNR_ERROR(  g_driver->logger_, "Error in ORLOPEN_set_ExtData." );
                  
                  if (  g_driver->absolute_target_jnt_position_log_[active_arm]->full() )
                     g_driver->absolute_target_jnt_position_log_[active_arm]->pop_front();
                  
                  g_driver->absolute_target_jnt_position_log_[active_arm]->push_back( g_driver->last_absolute_target_jnt_position_rcv_[active_arm] );  
                }
                else
                {                
                  if ( ORLOPEN_set_absolute_pos_target_degree( &g_driver->last_absolute_target_jnt_position_rcv_[active_arm].target_pos, ORL_SILENT, g_driver->c5gopen_ctrl_idx_, get_orl_arm_num(active_arm) ) != ORLOPEN_RES_OK )
                    CNR_ERROR(  g_driver->logger_, "Error in ORLOPEN_set_absolute_pos_target_degree." );
                  
                  ORL_axis_2_joints( &g_driver->last_absolute_target_jnt_position_rcv_[active_arm].target_pos, ORL_SILENT, g_driver->c5gopen_ctrl_idx_, get_orl_arm_num(active_arm) );
                  
                  g_driver->last_absolute_target_jnt_position_rcv_[active_arm].target_vel.unit_type = ORL_SPEED_LINK_DEGREE_SEC;
                  ORL_joints_conversion( &g_driver->last_absolute_target_jnt_position_rcv_[active_arm].target_vel, ORL_SPEED_MOTORROUNDS, ORL_SILENT, g_driver->c5gopen_ctrl_idx_, get_orl_arm_num(active_arm) );
                  
                  if ( ORLOPEN_set_ExtData( &g_driver->last_absolute_target_jnt_position_rcv_[active_arm].target_vel, &mask, ORL_SILENT, g_driver->c5gopen_ctrl_idx_, get_orl_arm_num(active_arm) ) != ORLOPEN_RES_OK )
                    CNR_ERROR(  g_driver->logger_, "Error in ORLOPEN_set_ExtData." );
                  
                  if (  g_driver->absolute_target_jnt_position_log_[active_arm]->full() )
                     g_driver->absolute_target_jnt_position_log_[active_arm]->pop_front();
                
                   g_driver->absolute_target_jnt_position_log_[active_arm]->push_back( g_driver->last_absolute_target_jnt_position_rcv_[active_arm] );
                  
                }
              }
              else
              {
                g_driver->starting_absolute_jnt_position_[active_arm].target_pos.unit_type = ORL_POSITION_LINK_DEGREE;
                
                if ( ORLOPEN_set_absolute_pos_target_degree( &g_driver->starting_absolute_jnt_position_[active_arm].target_pos, ORL_SILENT, g_driver->c5gopen_ctrl_idx_, get_orl_arm_num(active_arm) ) != ORLOPEN_RES_OK )
                  CNR_ERROR(  g_driver->logger_, "Error in ORLOPEN_set_absolute_pos_target_degree.");
                
                ORL_axis_2_joints( &g_driver->starting_absolute_jnt_position_[active_arm].target_pos , ORL_SILENT, g_driver->c5gopen_ctrl_idx_, get_orl_arm_num(active_arm) );   
                
                g_driver->starting_absolute_jnt_position_[active_arm].target_vel.unit_type = ORL_SPEED_LINK_DEGREE_SEC;
                ORL_joints_conversion( &g_driver->starting_absolute_jnt_position_[active_arm].target_vel, ORL_SPEED_MOTORROUNDS, ORL_SILENT, g_driver->c5gopen_ctrl_idx_, get_orl_arm_num(active_arm) );
                            
                if ( ORLOPEN_set_ExtData( &g_driver->starting_absolute_jnt_position_[active_arm].target_vel, &mask, ORL_SILENT, g_driver->c5gopen_ctrl_idx_, get_orl_arm_num(active_arm) ) != ORLOPEN_RES_OK )
                  CNR_ERROR(  g_driver->logger_, "Error in ORLOPEN_set_ExtData.");
                
                if (  g_driver->absolute_target_jnt_position_log_[active_arm]->full() )
                  g_driver->absolute_target_jnt_position_log_[active_arm]->pop_front();
              
                 g_driver->absolute_target_jnt_position_log_[active_arm]->push_back( g_driver->starting_absolute_jnt_position_[active_arm] );
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
      
      if ( g_driver->flag_ExitFromOpen_[active_arm] )
        ORLOPEN_ExitFromOpen( ORL_VERBOSE, g_driver->c5gopen_ctrl_idx_, get_orl_arm_num(active_arm) );
      
    }
    
    return ORLOPEN_RES_OK;
  }

} // end of namespace c5gopen