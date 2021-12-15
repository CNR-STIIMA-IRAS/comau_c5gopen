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

#ifndef __C5GOPEN_THREAD__
#define __C5GOPEN_THREAD__

#include <chrono>
#include <thread>

#include <cnr_logger/cnr_logger.h>

#define EXTRN  extern

// #define false       0
// #define true        1
// #define LAST_MESS   0

// #define MAX_NUM_ARMS    1          // ATTENTION: define here the number of ARM for ORL library

// #define target_pos_max_buff_len 100
// #define target_pos_max_buff_log_len 300
// #define delta_target_cart_pos_max_len 10

// struct AI_Input 
// {
//   int16_t   ch1_value[ 6 ];
// }__attribute__ ( ( packed ) );


// struct delta_position_t 
// {
//   ORL_cartesian_position ideal;
//   ORL_cartesian_position real;
//   ORL_cartesian_position speed;
// }__attribute__ ( ( packed ) );


// struct absolute_target_position_t
// {
//   ORL_joint_value target_pos;
//   ORL_joint_value target_vel;
// }__attribute__ ( ( packed ) ); 


// struct absolute_real_position_t
// {
//   ORL_joint_value real_pos;
//   ORL_joint_value real_vel;
// }__attribute__ ( ( packed ) ); 


// EXTRN int                         mask_moving_arms;
// EXTRN int                         cycle_active;
// EXTRN bool                        system_initialized;

// EXTRN bool                        first_arm_driveon             [MAX_NUM_ARMS];
// EXTRN bool                        flag_RunningMove              [MAX_NUM_ARMS];
// EXTRN bool                        flag_ExitFromOpen             [MAX_NUM_ARMS];
// EXTRN bool                        trajectory_in_execution       [MAX_NUM_ARMS];
// EXTRN bool                        received_delta_target_cart    [MAX_NUM_ARMS];
// EXTRN bool                        robot_movement_enabled        [MAX_NUM_ARMS];
// EXTRN unsigned int                modality_active               [MAX_NUM_ARMS];
// EXTRN unsigned int                modality_old                  [MAX_NUM_ARMS];

// EXTRN ORL_joint_value             actual_joints_position        [MAX_NUM_ARMS];
// EXTRN ORL_cartesian_position      actual_cartesian_position     [MAX_NUM_ARMS];

// EXTRN realtime_buffer::circ_buffer<delta_position_t> *delta_target_cart_pos[MAX_NUM_ARMS];

// EXTRN realtime_buffer::circ_buffer<absolute_target_position_t> *absolute_target_position[MAX_NUM_ARMS];
// EXTRN realtime_buffer::circ_buffer<absolute_target_position_t> *absolute_target_position_log[MAX_NUM_ARMS];



namespace c5gopen
{
  struct C5GOpenThreadSharedStruct 
  {  
    int period;
    bool c5gopen_active;
    char ip_ctrl[255];
    char sys_id[255];
    
    std::shared_ptr<cnr_logger::TraceLogger> logger;    
  };

  void *c5gopen_thread( void *shared_thread_data );

}
#endif