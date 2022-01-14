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
    return get_c5gopen_period_in_usec(c5gopen_period) * pow(10.0,3.0);
  }

}