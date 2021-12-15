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

#include <comau_c5gopen_lpc/c5gopen_callback.h>

namespace c5gopen
{
  int user_callback ( int input ) 
  {
    static bool first_entry = false;
    if( 0 )
    {
      static int policy_;
      static struct sched_param user_callback_thread_param;
      pthread_getschedparam( pthread_self(), &policy_, &user_callback_thread_param );
      
      user_callback_thread_param.sched_priority = 49; 
      
      if ( pthread_setschedparam( pthread_self(), policy_, &user_callback_thread_param ) != 0 )
        printf( " [ %s%s:%d%s ]\t %s ERROR in pthread_setschedparam() of usercallback_thread_id%s \n", GREEN, __FUNCFILE__, __LINE__, RESET, RED, RESET);
      
      first_entry = true;
    }
    
    long mask;
    bool arm_driveon;
    char s_modality [40];
    char flag_new_modality [MAX_NUM_ARMS];
    
    
    for ( int iArm=0; iArm<MAX_NUM_ARMS; iArm++ ) 
    {
      if ( !flag_ExitFromOpen[iArm] )
      {
        flag_new_modality   [iArm] = false;
        modality_old        [iArm] = modality_active[iArm];
        modality_active     [iArm] = ORLOPEN_GetModeMasterAx  ( ORL_SILENT,ORL_CNTRL01, iArm );
        mask                       = ORLOPEN_GetOpenMask      ( ORL_SILENT,ORL_CNTRL01, iArm );
        
        arm_driveon = ( ORLOPEN_GetStatusMasterAx(ORL_SILENT, ORL_CNTRL01, iArm) == CRCOPEN_STS_DRIVEON ) ? true : false;
        
        if ( arm_driveon && !first_arm_driveon[iArm] )
          first_arm_driveon[iArm] = true;
        
        decode_modality( (unsigned int)modality_active[iArm], s_modality, false );
            
        if( modality_old[iArm] != modality_active[iArm] ) 
        {
          flag_new_modality[iArm] = true;
          printf( " [ %s%s:%d%s ]\tARM %d Modality %d %s\n", GREEN, __FUNCFILE__, __LINE__, RESET, iArm+1, (unsigned int)modality_active[iArm], s_modality);
        }   
        else 
          flag_new_modality[iArm] = false;
        
        switch ( modality_active[iArm] ) 
        {
          // LISTEN MODE
          case CRCOPEN_LISTEN:
            if (system_initialized)
            { 
              if ( first_arm_driveon[iArm] )
              {
                if ( ORLOPEN_sync_position( &actual_joints_position[iArm], ORL_SILENT, ORL_CNTRL01, iArm ) < ORLOPEN_RES_OK )
                {
                  ORLOPEN_sync_position( &actual_joints_position[iArm], ORL_VERBOSE, ORL_CNTRL01, iArm );
                  printf( " [ %s%s:%d%s ]\t %serror in ORLOPEN_sync_position %s\n", GREEN, __FUNCFILE__, __LINE__, RESET, RED, RESET );
                  exit(0);
                }
              }
              
              ORL_joint_value         actual_joints_position_         [MAX_NUM_ARMS];
              ORL_cartesian_position  actual_cartesian_position_      [MAX_NUM_ARMS];
              
              if ( ORLOPEN_get_pos_measured(&actual_joints_position_[iArm],&actual_cartesian_position_[iArm], LAST_MESS, ORL_SILENT, ORL_CNTRL01, iArm ) < ORLOPEN_RES_OK )
              {
                ORLOPEN_get_pos_measured(&actual_joints_position_[iArm],&actual_cartesian_position_[iArm], LAST_MESS, ORL_VERBOSE, ORL_CNTRL01, iArm );
                printf( " [ %s%s:%d%s ]\t %serror in ORLOPEN_get_pos_measured%s\n", GREEN, __FUNCFILE__, __LINE__, RESET, RED, RESET );
                exit(0);
              }      
              
              memset( &starting_absolute_position[iArm], 0x00, sizeof(absolute_target_position_t) );
              memcpy( &starting_absolute_position[iArm], &actual_joints_position_[iArm], sizeof(absolute_target_position_t ));
              
              if ( absolute_target_position_log[iArm]->full() )
                absolute_target_position_log[iArm]->pop_front();

              absolute_target_position_log[iArm]->push_back( starting_absolute_position[iArm] );
                
            }
            
            break;

          // ABSOLUTE MODE     
          case CRCOPEN_POS_ABSOLUTE:
            if (cycle_active)
            {
              if ( (flag_new_modality[iArm]) && (modality_active[iArm] == CRCOPEN_POS_ABSOLUTE) )
              {
                printf( " [ %s%s:%d%s ]\tModality CRCOPEN_POS_ABSOLUTE activated \n", GREEN, __FUNCFILE__, __LINE__, RESET );
                robot_movement_enabled[iArm] = false;
              }
            }
            
            if ( arm_driveon )
            {
              if ( robot_movement_enabled[iArm] )
              {  
                if ( !absolute_target_position[iArm]->empty() )
                {
                  memcpy( &last_absolute_target_position_rcv[iArm], &absolute_target_position[iArm]->front(), sizeof(absolute_target_position_t ));
                  absolute_target_position[iArm]->pop_front();
                  
                  if ( ORLOPEN_set_absolute_pos_target_degree( &last_absolute_target_position_rcv[iArm].target_pos, ORL_SILENT, ORL_CNTRL01, iArm ) != ORLOPEN_RES_OK )
                    printf( " [ %s%s:%d%s ]\t %serror in ORLOPEN_set_absolute_pos_target_degree %s\n", GREEN, __FUNCFILE__, __LINE__, RESET, RED, RESET );
                  
                  ORL_axis_2_joints( &last_absolute_target_position_rcv[iArm].target_pos, ORL_SILENT, ORL_CNTRL01, iArm );

                  last_absolute_target_position_rcv[iArm].target_vel.unit_type = ORL_SPEED_LINK_DEGREE_SEC;
                  ORL_joints_conversion( &last_absolute_target_position_rcv[iArm].target_vel, ORL_SPEED_MOTORROUNDS, ORL_SILENT, ORL_CNTRL01, iArm );
                  
                  if ( ORLOPEN_set_ExtData( &last_absolute_target_position_rcv[iArm].target_vel, &mask, ORL_SILENT, ORL_CNTRL01, iArm ) != ORLOPEN_RES_OK )
                    printf( " [ %s%s:%d%s ]\t %serror in ORLOPEN_set_ExtData %s\n", GREEN, __FUNCFILE__, __LINE__, RESET, RED, RESET );
                  
                  if ( absolute_target_position_log[iArm]->full() )
                    absolute_target_position_log[iArm]->pop_front();
                  
                  absolute_target_position_log[iArm]->push_back( last_absolute_target_position_rcv[iArm] );  
                }
                else
                {                
                  if ( ORLOPEN_set_absolute_pos_target_degree( &last_absolute_target_position_rcv[iArm].target_pos, ORL_SILENT, ORL_CNTRL01, iArm ) != ORLOPEN_RES_OK )
                    printf( " [ %s%s:%d%s ]\t %serror in ORLOPEN_set_absolute_pos_target_degree %s\n", GREEN, __FUNCFILE__, __LINE__, RESET, RED, RESET );
                  
                  ORL_axis_2_joints( &last_absolute_target_position_rcv[iArm].target_pos, ORL_SILENT, ORL_CNTRL01, iArm );
                  
                  last_absolute_target_position_rcv[iArm].target_vel.unit_type = ORL_SPEED_LINK_DEGREE_SEC;
                  ORL_joints_conversion( &last_absolute_target_position_rcv[iArm].target_vel, ORL_SPEED_MOTORROUNDS, ORL_SILENT, ORL_CNTRL01, iArm );
                  
                  if ( ORLOPEN_set_ExtData( &last_absolute_target_position_rcv[iArm].target_vel, &mask, ORL_SILENT, ORL_CNTRL01, iArm ) != ORLOPEN_RES_OK )
                    printf( " [ %s%s:%d%s ]\t %serror in ORLOPEN_set_ExtData %s\n", GREEN, __FUNCFILE__, __LINE__, RESET, RED, RESET );
                  
                  if ( absolute_target_position_log[iArm]->full() )
                    absolute_target_position_log[iArm]->pop_front();
                
                  absolute_target_position_log[iArm]->push_back( last_absolute_target_position_rcv[iArm] );
                  
                }
              }
              else
              {
                starting_absolute_position[iArm].target_pos.unit_type = ORL_POSITION_LINK_DEGREE;
                
                if ( ORLOPEN_set_absolute_pos_target_degree( &starting_absolute_position[iArm].target_pos, ORL_SILENT, ORL_CNTRL01, iArm ) != ORLOPEN_RES_OK )
                    printf( " [ %s%s:%d%s ]\t %serror in ORLOPEN_set_absolute_pos_target_degree %s\n", GREEN, __FUNCFILE__, __LINE__, RESET, RED, RESET );
                
                ORL_axis_2_joints( &starting_absolute_position[iArm].target_pos , ORL_SILENT, ORL_CNTRL01, iArm );   
                
                starting_absolute_position[iArm].target_vel.unit_type = ORL_SPEED_LINK_DEGREE_SEC;
                ORL_joints_conversion( &starting_absolute_position[iArm].target_vel, ORL_SPEED_MOTORROUNDS, ORL_SILENT, ORL_CNTRL01, iArm );
                            
                if ( ORLOPEN_set_ExtData( &starting_absolute_position[iArm].target_vel, &mask, ORL_SILENT, ORL_CNTRL01, iArm ) != ORLOPEN_RES_OK )
                    printf( " [ %s%s:%d%s ]\t %serror in ORLOPEN_set_ExtData %s\n", GREEN, __FUNCFILE__, __LINE__, RESET, RED, RESET );
                
                if ( absolute_target_position_log[iArm]->full() )
                  absolute_target_position_log[iArm]->pop_front();
              
                absolute_target_position_log[iArm]->push_back( starting_absolute_position[iArm] );
              }
              
            }
            
            break;
            
          // ADDITIVE MODE   
          case CRCOPEN_POS_ADDITIVE:
            if (cycle_active)
            {
              if ( (flag_new_modality[iArm]) && (modality_active[iArm] == CRCOPEN_POS_ADDITIVE) )
              {
                printf( " [ %s%s:%d%s ]\tModality CRCOPEN_POS_ADDITIVE activated \n", GREEN, __FUNCFILE__, __LINE__, RESET );
                received_delta_target_cart[iArm] = false;
              }
            }
            
            if ( arm_driveon )
            {
              if ( received_delta_target_cart[iArm] )
              {  
                if ( !delta_target_cart_pos[iArm]->empty() )
                {
                  memcpy( &last_delta_target_cart_pos[iArm], &delta_target_cart_pos[iArm]->front(), sizeof(delta_position_t ));
                  delta_target_cart_pos[iArm]->pop_front();                  
                }
              }
              else
                memset( &last_delta_target_cart_pos[iArm].ideal, 0x00, sizeof(delta_position_t) );
              
              if ( ORLOPEN_get_pos_measured(&additive_starting_jnt_link_[iArm], &additive_starting_cartesian_pose_[iArm], LAST_MESS, ORL_SILENT, ORL_CNTRL01, iArm ) < ORLOPEN_RES_OK )
              {
                ORLOPEN_get_pos_measured(&additive_starting_jnt_link_[iArm],&additive_starting_cartesian_pose_[iArm], LAST_MESS, ORL_VERBOSE, ORL_CNTRL01, iArm );
                printf( " [ %s%s:%d%s ]\t %serror in ORLOPEN_get_pos_measured%s\n", GREEN, __FUNCFILE__, __LINE__, RESET, RED, RESET );
                exit(0);
              }
              
              last_delta_target_cart_pos[iArm].ideal.unit_type = ORL_CART_POSITION;
              strcpy( last_delta_target_cart_pos[iArm].ideal.config_flags, additive_starting_cartesian_pose_[iArm].config_flags);
              
              if ( ORLOPEN_set_additive_pos_target_cartesian( &last_delta_target_cart_pos[iArm].ideal, ORL_SILENT, ORL_CNTRL01, iArm ) != ORLOPEN_RES_OK )
                printf( " [ %s%s:%d%s ]\t %serror in ORLOPEN_set_additive_pos_target_cartesian %s\n", GREEN, __FUNCFILE__, __LINE__, RESET, RED, RESET ); 
                          
            }

            break;
            
          default:
            break;
        
        }
      }
      
      if ( flag_ExitFromOpen[iArm] )
        ORLOPEN_ExitFromOpen( ORL_SILENT,  ORL_CNTRL01, iArm );
      
    }
    
    return ORLOPEN_RES_OK;
  }

}
