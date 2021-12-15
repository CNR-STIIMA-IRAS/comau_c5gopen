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

#include <eORL.h>

#include <comau_c5gopen_lpc/c5gopen_thread.h>
#include <comau_c5gopen_lpc/c5gopen_utilities.h>
#include <comau_c5gopen_lpc/c5gopen_callback.h>

namespace c5gopen
{
  void *c5gopen_thread( void* shared_thread_data )
  {
    c5gopen::C5GOpenThreadSharedStruct* ptr_shared_data = reinterpret_cast<c5gopen::C5GOpenThreadSharedStruct *>( shared_thread_data );
    
    CNR_INFO(*ptr_shared_data->logger, "C5Gopen theread started. ");

  // bool                        first_arm_driveon             [MAX_NUM_ARMS] = {false};
  // bool                        flag_RunningMove              [MAX_NUM_ARMS] = {false};
  // bool                        flag_ExitFromOpen             [MAX_NUM_ARMS] = {false};
  // bool                        trajectory_in_execution       [MAX_NUM_ARMS] = {false};
  // bool                        received_delta_target_cart    [MAX_NUM_ARMS] = {false};
  // bool                        robot_movement_enabled        [MAX_NUM_ARMS] = {false};
  // unsigned int                modality_active               [MAX_NUM_ARMS] = {0x0};
  // unsigned int                modality_old                  [MAX_NUM_ARMS] = {0x0};

  // for( int iArm=0; iArm<MAX_NUM_ARMS; iArm++ )
  // {
  //   flag_RunningMove            [iArm] = false;
  //   flag_ExitFromOpen           [iArm] = false;
  //   modality_active             [iArm] = CRCOPEN_LISTEN;
  //   modality_old                [iArm] = CRCOPEN_LISTEN;
  //   robot_movement_enabled      [iArm] = false;
  //   trajectory_in_execution     [iArm] = false;
  //   received_delta_target_cart  [iArm] = false;
    
  //   memset( &actual_joints_position[iArm], 0x00, sizeof(ORL_joint_value));
  //   memset( &actual_cartesian_position[iArm], 0x00, sizeof(ORL_cartesian_position));

  //   delta_target_cart_pos[iArm] = new itia::butils::circ_buffer<delta_position_t>(delta_target_cart_pos_max_len);
  //   absolute_target_position[iArm] = new itia::butils::circ_buffer<absolute_target_position_t>(target_pos_max_buff_len);
  //   absolute_target_position_log[iArm] = new itia::butils::circ_buffer<absolute_target_position_t>(target_pos_max_buff_log_len);
    
  //   c5gopen_exec[iArm] = new C5gopenTrjExec( iArm+1, period, trj_namespace, &nh, delta_target_cart_pos[iArm], absolute_target_position[iArm] );
    
  // }

    if( ORLOPEN_initialize_controller ( ptr_shared_data->ip_ctrl, ptr_shared_data->sys_id, ORL_SILENT, ORL_CNTRL01) != ORLOPEN_RES_OK )
    {
      ORLOPEN_initialize_controller ( ptr_shared_data->ip_ctrl, ptr_shared_data->sys_id, ORL_VERBOSE, ORL_CNTRL01);
      CNR_ERROR( *ptr_shared_data->logger, "Error in ORL_initialize_controller " );
      exit(0);
    }
    else 
      CNR_INFO( *ptr_shared_data->logger, std::string(ptr_shared_data->ip_ctrl) + ": " +
                                          std::string(ptr_shared_data->sys_id) + ".c5g OK" );
        
    if ( ORLOPEN_set_period( ptr_shared_data->period, ORL_SILENT, ORL_CNTRL01 ) != ORLOPEN_RES_OK )
    {
      ORLOPEN_set_period( ptr_shared_data->period, ORL_VERBOSE, ORL_CNTRL01 );
      CNR_ERROR( *ptr_shared_data->logger, "Error in ORLOPEN_set_period" );
      exit(0);
    }
    
    ORL_cartesian_position bFrame, tFrame, uFrame;
    if ( !set_frames( &bFrame, &tFrame, &uFrame ) )
    {
      CNR_ERROR( *ptr_shared_data->logger, "Error cannot set frames." );
      exit(0);
    }
    
    if ( ORL_initialize_frames( bFrame, tFrame, uFrame, ORL_SILENT, ORL_CNTRL01, ORL_ARM1 ) != ORLOPEN_RES_OK )  
    {
      ORL_initialize_frames( bFrame, tFrame, uFrame, ORL_VERBOSE, ORL_CNTRL01, ORL_ARM1 );
      CNR_ERROR(  *ptr_shared_data->logger, "Error in ORL_initialize_frames." );
      exit(0);
    } 
      
    if ( ORLOPEN_SetCallBackFunction( &user_callback, ORL_SILENT, ORL_CNTRL01 ) < ORLOPEN_RES_OK )
    {
      ORLOPEN_SetCallBackFunction( &user_callback, ORL_VERBOSE, ORL_CNTRL01 );
      CNR_ERROR( *ptr_shared_data->logger, "Error in ORLOPEN_SetCallBackFunction.");
      exit(0);
    }
    else
      CNR_INFO( *ptr_shared_data->logger, "User callback function initialized.");  
    
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    /******************** Start with C5Gopen control *******************/

    if ( ORLOPEN_StartCommunication( ORL_SILENT ) != ORLOPEN_RES_OK )
    {
      ORLOPEN_StartCommunication( ORL_VERBOSE );
      ORLOPEN_GetPowerlinkState( ORL_VERBOSE );
      exit(0);
    }
    
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        
    if ( initialize_control_position() < 0 ) 
    {
      CNR_ERROR( *ptr_shared_data->logger, "Error in initialize_control_position() function.");
      exit(0);
    }

    CNR_INFO( *ptr_shared_data->logger, "C5Gopen theread started. ");

    // Enter in the infinite loop
    while (ptr_shared_data->c5gopen_active)
    {
      std::this_thread::sleep_for(std::chrono::microseconds(400)); // need to use varibale to set sleep_for function
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    ORLOPEN_StopCommunication( ORL_VERBOSE );

    std::this_thread::sleep_for(std::chrono::milliseconds(4000));
    ORL_terminate_controller( ORL_VERBOSE, ORL_CNTRL01 );



    CNR_INFO( *ptr_shared_data->logger, "C5Gopen theread properly stopped. ");
  }
}