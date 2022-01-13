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

#include <comau_c5gopen_lpc/c5gopen_utilities.h>
#include <comau_c5gopen_lpc/realtime_utilities.h>
#include <comau_c5gopen_lpc/c5gopen_driver.h>

namespace c5gopen
{
  C5GOpenDriver* g_driver;

  void init_driver_library(C5GOpenDriver* c5gopen_driver){
    g_driver = c5gopen_driver;
  }

  C5GOpenDriver::C5GOpenDriver( const std::string& ip_ctrl, const std::string& sys_id,
                                const int& c5gopen_period, std::shared_ptr<cnr_logger::TraceLogger>& logger ): 
                                ip_ctrl_(ip_ctrl), sys_id_(sys_id), c5gopen_period_(c5gopen_period), logger_(logger)
  {
    bool threads_status_ = true;
    bool c5gopen_active_ = true;
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
    c5gopen_thread_.join();
    com_thread_.join();
    loop_console_thread_.join();
    
    // something else to be done here
    
    return true;
  };

  bool C5GOpenDriver::getThreadsStatus()
  {
    return threads_status_;
  }

  // C5G Open thread
  void C5GOpenDriver::c5gopen_thread()
  {
#if PREEMPTIVE_RT
    realtime_utilities::period_info  pinfo;
    if(!realtime_utilities::rt_init_thread(RT_STACK_SIZE, sched_get_priority_max(SCHED_OTHER), SCHED_OTHER, &pinfo, (long)get_c5gopen_period_in_nsec(c5gopen_period_)) )
    {
      CNR_ERROR( *logger_, "Failed in setting thread rt properties. Exit. ");
      std::raise(SIGINT);
      return;
    }
#endif

    CNR_INFO( *logger_, "C5Gopen thread started. " );

    if( ORLOPEN_initialize_controller ( ip_ctrl_.c_str(), sys_id_.c_str(), ORL_SILENT, ORL_CNTRL01) != ORLOPEN_RES_OK )
    {
      ORLOPEN_initialize_controller ( ip_ctrl_.c_str(), sys_id_.c_str(), ORL_VERBOSE, ORL_CNTRL01);
      CNR_ERROR( *logger_, "Error in ORL_initialize_controller " );
      exit(0);
    }
    else 
      CNR_INFO( *logger_, std::string(ip_ctrl_) + ": " + std::string(sys_id_) + ".c5g OK" );
        
    if ( ORLOPEN_set_period( c5gopen_period_, ORL_SILENT, ORL_CNTRL01 ) != ORLOPEN_RES_OK )
    {
      ORLOPEN_set_period( c5gopen_period_, ORL_VERBOSE, ORL_CNTRL01 );
      CNR_ERROR( *logger_, "Error in ORLOPEN_set_period" );
      exit(0);
    }
    
    ORL_cartesian_position bFrame, tFrame, uFrame;
    if ( !c5gopen::set_frames( &bFrame, &tFrame, &uFrame ) )
    {
      CNR_ERROR( *logger_, "Error cannot set frames." );
      exit(0);
    }
    
    if ( ORL_initialize_frames( bFrame, tFrame, uFrame, ORL_SILENT, ORL_CNTRL01, ORL_ARM1 ) != ORLOPEN_RES_OK )  
    {
      ORL_initialize_frames( bFrame, tFrame, uFrame, ORL_VERBOSE, ORL_CNTRL01, ORL_ARM1 );
      CNR_ERROR(  *logger_, "Error in ORL_initialize_frames." );
      exit(0);
    } 

    ORLOPEN_callback callback_comau; 

    if ( ORLOPEN_SetCallBackFunction( c5gopen_callback, ORL_SILENT, ORL_CNTRL01 ) < ORLOPEN_RES_OK )
    {
      ORLOPEN_SetCallBackFunction( c5gopen_callback, ORL_VERBOSE, ORL_CNTRL01 );
      CNR_ERROR( *logger_, "Error in ORLOPEN_SetCallBackFunction.");
      exit(0);
    }
    else
      CNR_INFO( *logger_, "User callback function initialized.");  
    
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    /******************** Start with C5Gopen control *******************/

    if ( ORLOPEN_StartCommunication( ORL_SILENT ) != ORLOPEN_RES_OK )
    {
      ORLOPEN_StartCommunication( ORL_VERBOSE );
      ORLOPEN_GetPowerlinkState( ORL_VERBOSE );
      exit(0);
    }
    
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        
    if ( c5gopen::initialize_control_position() < 0 ) 
    {
      CNR_ERROR( *logger_, "Error in initialize_control_position() function.");
      exit(0);
    }

    CNR_INFO( *logger_, "C5Gopen theread started. ");

    // Enter in the infinite loop
    while (c5gopen_active_)
    {
      std::this_thread::sleep_for(std::chrono::microseconds(1000)); // need to use varibale to set sleep_for function
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    ORLOPEN_StopCommunication( ORL_VERBOSE );

    std::this_thread::sleep_for(std::chrono::milliseconds(4000));
    ORL_terminate_controller( ORL_VERBOSE, ORL_CNTRL01 );

    CNR_INFO( *logger_, "C5Gopen theread properly stopped. ");
  }

  // C5G Open thread
  void C5GOpenDriver::com_thread()
  {
    // to be done
  }

  // Console thread
  void C5GOpenDriver::loop_console_thread()
  {
    // to be done
  }


  int c5gopen_callback ( int input ) 
  {
    // static bool first_entry = false;
    // if( 0 )
    // {
    //   static int policy_;
    //   static struct sched_param user_callback_thread_param;
    //   pthread_getschedparam( pthread_self(), &policy_, &user_callback_thread_param );
      
    //   user_callback_thread_param.sched_priority = 49; 
      
    //   if ( pthread_setschedparam( pthread_self(), policy_, &user_callback_thread_param ) != 0 )
    //     printf( " [ %s%s:%d%s ]\t %s ERROR in pthread_setschedparam() of usercallback_thread_id%s \n", GREEN, __FUNCFILE__, __LINE__, RESET, RED, RESET);
      
    //   first_entry = true;
    // }
    
    long mask;
    bool arm_driveon;
    char s_modality[40];
    char flag_new_modality[MAX_NUM_ARMS];
    
    
    for ( int iArm=0; iArm<MAX_NUM_ARMS; iArm++ ) 
    {
      if ( !g_driver->flag_ExitFromOpen_[iArm] )
      {
        flag_new_modality[iArm]           = false;
        g_driver->modality_old_[iArm]     = g_driver->modality_active_[iArm];
        
        mask = ORLOPEN_GetOpenMask      ( ORL_SILENT,ORL_CNTRL01, iArm );
        g_driver->modality_active_[iArm]  = ORLOPEN_GetModeMasterAx  ( ORL_SILENT,ORL_CNTRL01, iArm );
        
        
        arm_driveon = ( ORLOPEN_GetStatusMasterAx(ORL_SILENT, ORL_CNTRL01, iArm) == CRCOPEN_STS_DRIVEON ) ? true : false;
        
        if ( arm_driveon && ! g_driver->first_arm_driveon_[iArm] )
           g_driver->first_arm_driveon_[iArm] = true;
        
        decode_modality( (unsigned int) g_driver->modality_active_[iArm], s_modality, false );
            
        if(  g_driver->modality_old_[iArm] !=  g_driver->modality_active_[iArm] ) 
        {
          flag_new_modality[iArm] = true;
          CNR_INFO( g_driver->logger_, "ARM " << iArm+1 << " Modality" << (unsigned int)g_driver->modality_active_[iArm] << " " <<  s_modality );
        }   
        else 
          flag_new_modality[iArm] = false;
        
        switch (  g_driver->modality_active_[iArm] ) 
        {
          // LISTEN MODE
          case CRCOPEN_LISTEN:
            if ( g_driver->system_initialized_)
            { 
              if (  g_driver->first_arm_driveon_[iArm] )
              {
                if ( ORLOPEN_sync_position( & g_driver->actual_joints_position_[iArm], ORL_SILENT, ORL_CNTRL01, iArm ) < ORLOPEN_RES_OK )
                {
                  ORLOPEN_sync_position( & g_driver->actual_joints_position_[iArm], ORL_VERBOSE, ORL_CNTRL01, iArm );
                  CNR_ERROR(  g_driver->logger_, "Error in ORLOPEN_sync_position.");
                  exit(0);
                }
              }
              
              ORL_joint_value actual_joints_position_tmp [MAX_NUM_ARMS];
              ORL_cartesian_position actual_cartesian_position_tmp [MAX_NUM_ARMS];
              
              if ( ORLOPEN_get_pos_measured(&actual_joints_position_tmp[iArm],&actual_cartesian_position_tmp[iArm], LAST_MESS, ORL_SILENT, ORL_CNTRL01, iArm ) < ORLOPEN_RES_OK )
              {
                ORLOPEN_get_pos_measured(&actual_joints_position_tmp[iArm],&actual_cartesian_position_tmp[iArm], LAST_MESS, ORL_VERBOSE, ORL_CNTRL01, iArm );
                CNR_ERROR(  g_driver->logger_, "Error in ORLOPEN_get_pos_measured.");
                exit(0);
              }      
              
              memset( &g_driver->starting_absolute_jnt_position_[iArm], 0x00, sizeof(absolute_target_position_t) );
              memcpy( &g_driver->starting_absolute_jnt_position_[iArm], & g_driver->actual_joints_position_[iArm], sizeof(absolute_target_position_t ));
              
              if (  g_driver->absolute_target_jnt_position_log_[iArm].full() )
                g_driver-> absolute_target_jnt_position_log_[iArm].pop_front();

               g_driver->absolute_target_jnt_position_log_[iArm].push_back( g_driver->starting_absolute_jnt_position_[iArm] );
                
            }
            
            break;

          // ABSOLUTE MODE     
          case CRCOPEN_POS_ABSOLUTE:
            if (cycle_active_)
            {
              if ( (flag_new_modality[iArm]) && ( g_driver->modality_active_[iArm] == CRCOPEN_POS_ABSOLUTE) )
              {
                CNR_INFO(  g_driver->logger_, "Modality CRCOPEN_POS_ABSOLUTE activated." );
                 g_driver->robot_movement_enabled_[iArm] = false;
              }
            }
            
            if ( arm_driveon )
            {
              if (  g_driver->robot_movement_enabled_[iArm] )
              {  
                if ( ! g_driver->absolute_target_jnt_position_[iArm].empty() )
                {
                  memcpy( &g_driver->last_absolute_target_jnt_position_rcv_[iArm], & g_driver->absolute_target_jnt_position_[iArm].front(), sizeof(absolute_target_position_t ));
                   g_driver->absolute_target_jnt_position_[iArm].pop_front();
                  
                  if ( ORLOPEN_set_absolute_pos_target_degree( &g_driver->last_absolute_target_jnt_position_rcv_[iArm].target_pos, ORL_SILENT, ORL_CNTRL01, iArm ) != ORLOPEN_RES_OK )
                    CNR_ERROR(  g_driver->logger_, "Error in ORLOPEN_set_absolute_pos_target_degree." );
                  
                  ORL_axis_2_joints( &g_driver->last_absolute_target_jnt_position_rcv_[iArm].target_pos, ORL_SILENT, ORL_CNTRL01, iArm );

                  g_driver->last_absolute_target_jnt_position_rcv_[iArm].target_vel.unit_type = ORL_SPEED_LINK_DEGREE_SEC;
                  ORL_joints_conversion( &last_absolute_target_jnt_position_rcv_[iArm].target_vel, ORL_SPEED_MOTORROUNDS, ORL_SILENT, ORL_CNTRL01, iArm );
                  
                  if ( ORLOPEN_set_ExtData( &g_driver->last_absolute_target_jnt_position_rcv_[iArm].target_vel, &mask, ORL_SILENT, ORL_CNTRL01, iArm ) != ORLOPEN_RES_OK )
                    CNR_ERROR(  g_driver->logger_, "Error in ORLOPEN_set_ExtData." );
                  
                  if (  g_driver->absolute_target_jnt_position_log_[iArm].full() )
                     g_driver->absolute_target_jnt_position_log_[iArm].pop_front();
                  
                  g_driver->absolute_target_jnt_position_log_[iArm].push_back( g_driver->last_absolute_target_jnt_position_rcv_[iArm] );  
                }
                else
                {                
                  if ( ORLOPEN_set_absolute_pos_target_degree( &g_driver->last_absolute_target_jnt_position_rcv_[iArm].target_pos, ORL_SILENT, ORL_CNTRL01, iArm ) != ORLOPEN_RES_OK )
                    CNR_ERROR(  g_driver->logger_, "Error in ORLOPEN_set_absolute_pos_target_degree." );
                  
                  ORL_axis_2_joints( &g_driver->last_absolute_target_jnt_position_rcv_[iArm].target_pos, ORL_SILENT, ORL_CNTRL01, iArm );
                  
                  g_driver->last_absolute_target_jnt_position_rcv_[iArm].target_vel.unit_type = ORL_SPEED_LINK_DEGREE_SEC;
                  ORL_joints_conversion( &g_driver->last_absolute_target_jnt_position_rcv_[iArm].target_vel, ORL_SPEED_MOTORROUNDS, ORL_SILENT, ORL_CNTRL01, iArm );
                  
                  if ( ORLOPEN_set_ExtData( &g_driver->last_absolute_target_jnt_position_rcv_[iArm].target_vel, &mask, ORL_SILENT, ORL_CNTRL01, iArm ) != ORLOPEN_RES_OK )
                    CNR_ERROR(  g_driver->logger_, "Error in ORLOPEN_set_ExtData." );
                  
                  if (  g_driver->absolute_target_jnt_position_log_[iArm].full() )
                     g_driver->absolute_target_jnt_position_log_[iArm].pop_front();
                
                   g_driver->absolute_target_jnt_position_log_[iArm].push_back( last_absolute_target_jnt_position_rcv[iArm] );
                  
                }
              }
              else
              {
                g_driver->starting_absolute_jnt_position_[iArm].target_pos.unit_type = ORL_POSITION_LINK_DEGREE;
                
                if ( ORLOPEN_set_absolute_pos_target_degree( &g_driver->starting_absolute_jnt_position_[iArm].target_pos, ORL_SILENT, ORL_CNTRL01, iArm ) != ORLOPEN_RES_OK )
                  CNR_ERROR(  g_driver->logger_, "Error in ORLOPEN_set_absolute_pos_target_degree.");
                
                ORL_axis_2_joints( &g_driver->starting_absolute_jnt_position_[iArm].target_pos , ORL_SILENT, ORL_CNTRL01, iArm );   
                
                g_driver->starting_absolute_jnt_position_[iArm].target_vel.unit_type = ORL_SPEED_LINK_DEGREE_SEC;
                ORL_joints_conversion( &g_driver->starting_absolute_jnt_position_[iArm].target_vel, ORL_SPEED_MOTORROUNDS, ORL_SILENT, ORL_CNTRL01, iArm );
                            
                if ( ORLOPEN_set_ExtData( &g_driver->starting_absolute_jnt_position_[iArm].target_vel, &mask, ORL_SILENT, ORL_CNTRL01, iArm ) != ORLOPEN_RES_OK )
                  CNR_ERROR(  g_driver->logger_, "Error in ORLOPEN_set_ExtData.");
                
                if (  g_driver->absolute_target_jnt_position_log_[iArm].full() )
                  g_driver->absolute_target_jnt_position_log_[iArm].pop_front();
              
                 g_driver->absolute_target_jnt_position_log_[iArm].push_back( g_driver->starting_absolute_jnt_position_[iArm] );
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
      
      if ( g_driver->flag_ExitFromOpen[iArm] )
        ORLOPEN_ExitFromOpen( ORL_SILENT,  ORL_CNTRL01, iArm );
      
    }
    
    return ORLOPEN_RES_OK;
  }

} // end of namespace c5gopen