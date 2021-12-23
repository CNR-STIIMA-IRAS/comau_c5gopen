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

#include <math.h>
#include <stdio.h>

#include <comau_c5gopen_lpc/c5gopen_utilities.h>

namespace c5gopen
{

  bool set_frames( ORL_cartesian_position* bFrame, ORL_cartesian_position* tFrame, ORL_cartesian_position* uFrame )
  {
    /// TO BE DONE!!!!!! The funciton is currently only a mockup, the parameters should be taken from .yaml file
     
    bFrame->unit_type = ORL_CART_POSITION;
    bFrame->x = 0.0;
    bFrame->y = 0.0;
    bFrame->z = 0.0;
    bFrame->a = 0.0;
    bFrame->e = 0.0;
    bFrame->r = 0.0;
      
    tFrame->unit_type = ORL_CART_POSITION;
    tFrame->x = 0.0;
    tFrame->y = 0.0;
    tFrame->z = 0.0;
    tFrame->a = 0.0;
    tFrame->e = 0.0;
    tFrame->r = 0.0; 
    
    uFrame->unit_type = ORL_CART_POSITION;     
    uFrame->x = 0.0;
    uFrame->y = 0.0;
    uFrame->z = 0.0;
    uFrame->a = 0.0;
    uFrame->e = 0.0;
    uFrame->r = 0.0;
    
    return true;
    
  }

  int initialize_control_position ( void )
  {
    /// TO BE DONE!!!!

    // int modality_;
    // long output_jntmask_;
    // char s_modality_[40];
    // ORL_System_Variable orl_sys_var;

    // if ( ORLOPEN_GetPowerlinkState(ORL_VERBOSE) == PWL_ACTIVE )
    // {
    //   for ( int iArm=0; iArm<MAX_NUM_ARMS; iArm++ )
    //   {
    //     modality_        = ORLOPEN_GetModeMasterAx( ORL_SILENT, ORL_CNTRL01, iArm );
    //     output_jntmask_  = ORLOPEN_GetOpenMask( ORL_SILENT,ORL_CNTRL01, iArm );
        
    //     decode_modality( modality_, s_modality_, false );
        
    //     CNR_INFO ("\n------ ARM %d MODE %d %s - %s mask %x ------------ \n", 
    //             iArm+1,
    //             modality_,
    //             s_modality_,
    //           ( ( ORLOPEN_GetStatusMasterAx(ORL_SILENT, ORL_CNTRL01, iArm) == CRCOPEN_STS_DRIVEON ) ? "DRIVE_ON" : "DRIVEOFF"),
    //           (unsigned int)output_jntmask_ );
        
    //     if ( ORLOPEN_GetStatusMasterAx(ORL_SILENT, ORL_CNTRL01, iArm) != CRCOPEN_STS_DRIVEON )
    //       printf(" [ %s%s:%d%s ]\t %s ATTENTION: The system is in DRIVEOFF status, first syncronized position can't be reliable, don't forget the DRIVEON! %s\n", GREEN, __FUNCFILE__, __LINE__, RESET, BOLDYELLOW, RESET );
        
    //     if (first_arm_driveon[iArm])
    //     {
    //       ORLOPEN_sync_position(&actual_joints_position[iArm], ORL_SILENT, ORL_CNTRL01, iArm);
    //     }
        
    //     ORL_direct_kinematics(&actual_cartesian_position[iArm],&actual_joints_position[iArm],ORL_SILENT, ORL_CNTRL01,iArm);
        
    //     CNR_INFO(" [ %s%s:%d%s ]\t ORLOPEN_sync_position Joint %f %f %f %f %f %f %f %f %f %d\n",  GREEN, __FUNCFILE__, __LINE__, RESET, 
    //                                                                                             actual_joints_position[iArm].value[ORL_AX1],
    //                                                                                             actual_joints_position[iArm].value[ORL_AX2],
    //                                                                                             actual_joints_position[iArm].value[ORL_AX3],
    //                                                                                             actual_joints_position[iArm].value[ORL_AX4],
    //                                                                                             actual_joints_position[iArm].value[ORL_AX5],
    //                                                                                             actual_joints_position[iArm].value[ORL_AX6],
    //                                                                                             actual_joints_position[iArm].value[ORL_AX7],
    //                                                                                             actual_joints_position[iArm].value[ORL_AX8],
    //                                                                                             actual_joints_position[iArm].value[ORL_AX9],
    //                                                                                             (int)actual_joints_position[iArm].unit_type);
        
    //     printf( " [ %s%s:%d%s ]\t                       Pose  %f %f %f %f %f %f %s\n",  GREEN, __FUNCFILE__, __LINE__, RESET, 
    //                                                                                     actual_cartesian_position[iArm].x,
    //                                                                                     actual_cartesian_position[iArm].y,
    //                                                                                     actual_cartesian_position[iArm].z,
    //                                                                                     actual_cartesian_position[iArm].a,
    //                                                                                     actual_cartesian_position[iArm].e,
    //                                                                                     actual_cartesian_position[iArm].r,
    //                                                                                     actual_cartesian_position[iArm].config_flags);
        
    //     sprintf((char *)orl_sys_var.sysvar_name,"$ARM_DATA[%d].ARM_OVR",iArm+1);
        
    //     orl_sys_var.ctype = ORL_INT;
    //     orl_sys_var.iv = 20;
    //     ORL_set_data( orl_sys_var, ORL_SILENT, ORL_CNTRL01 );
    //   }
    //   system_initialized = true;
    // }
    // else
    //   return 1;
    
    return 0;
  }

  //****************************************************// 

  void decode_modality( const int& si_modality, char* string, const bool verbose )
  {
    switch(si_modality)
    {
    case CRCOPEN_LISTEN:
      sprintf(string,"CRCOPEN_LISTEN");
      break;
    case CRCOPEN_POS_ABSOLUTE:
      sprintf(string,"CRCOPEN_POS_ABSOLUTE");
      break;
    case CRCOPEN_POS_RELATIVE:
      sprintf(string,"CRCOPEN_POS_RELATIVE");
      break;
    case CRCOPEN_POS_ADDITIVE:
      sprintf(string,"CRCOPEN_POS_ADDITIVE");
      break;
    case CRCOPEN_POS_ADDITIVE_SB:
      sprintf(string,"CRCOPEN_POS_ADDITIVE_SB");
      break;
    case CRCOPEN_POS_ADDITIVE_SBE:
      sprintf(string,"CRCOPEN_POS_ADDITIVE_SBE");
      break;
    default:
      sprintf(string,"--");
      break;
    }
  }

  double get_c5gopen_period_in_usec( const int& c5gopen_period )
  {
    double period;
    switch(c5gopen_period)
    {
      case ORL_0_4_MILLIS:
        period = 400.0;
        break;
      case ORL_2_0_MILLIS:
        period = 2000.0;
        break;
      case ORL_4_0_MILLIS:
        period = 4000.0;
        break;
      case ORL_8_0_MILLIS:
        period = 8000.0;
        break;
      case ORL_16_0_MILLIS:
        period = 16000.0;
        break;
      default:
        period = 0.0;
        break;
      }
    return period;
  }

  double get_c5gopen_period_in_nsec( const int& c5gopen_period )
  {
     double period = get_c5gopen_period_in_usec(c5gopen_period) * pow(10.0,3.0);
  }

}