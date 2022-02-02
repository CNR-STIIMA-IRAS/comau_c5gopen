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

#ifndef C5GOPEN_UTILITIES_H
#define C5GOPEN_UTILITIES_H

#include <eORL.h>

#include <yaml-cpp/yaml.h>

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


  struct RobotJointStateArray
  {
    // Joint position must be in [deg] and velocities in [deg/s]
    double real_pos[ORL_MAX_AXIS] = {0};
    double real_vel[ORL_MAX_AXIS] = {0};
    double target_pos[ORL_MAX_AXIS] = {0};
    double target_vel[ORL_MAX_AXIS] = {0};
  }__attribute__ ( ( packed ) ); 

  struct RobotCartStateArray
  {
    // Cartesian position must be in [mm] and angles in [deg]
    double real_pos[6] = {0};
    char config_flags_real[80] = {0};
    double target_pos[6] = {0};
    char config_flags_target[80] = {0};
  }__attribute__ ( ( packed ) ); 

  struct RobotGenericArray
  {
    double value[ORL_MAX_AXIS] = {0};
  }__attribute__ ( ( packed ) );

  enum thread_status
  {
    BEFORE_RUN  = 0,
    RUNNING     = 1,
    CLOSED      = 2 
  }__attribute__ ( ( packed ) );

  struct C5GOpenDriverCfg
  {
    size_t ctrl_idx_orl_;
    std::string ip_ctrl_;
    std::string sys_id_;

    size_t c5gopen_period_orl_;
    
    std::vector<size_t> active_arms_;

    ORL_cartesian_position base_frame_;
    ORL_cartesian_position user_frame_;
    ORL_cartesian_position tool_frame_;
  };
  struct MQTTCfg
  {
    std::string mqtt_client_id_;
    std::string mqtt_broker_address_;
    size_t mqtt_port_;
    std::vector<std::string> mqtt_sub_topics_;
  };
  struct C5GOpenNodeCfg 
  {
    C5GOpenDriverCfg c5gopen_driver_cfg_;
    MQTTCfg mqtt_cfg_;
    std::string cnr_logger_cfg_file;
  };

  bool load_c5gopen_parameters( const std::string& config_file_name, C5GOpenNodeCfg& c5gopen_cfg );
  size_t get_orl_ctrl_num( const size_t& ctrl_idx );
  size_t get_orl_arm_num( const size_t& arm_idx );
  bool set_frames( const std::vector<double>& frame, ORL_cartesian_position& orl_frame );
  bool decode_modality( const int& si_modality, std::string& string );
  bool decode_c5gopen_frequency( const double& c5gopen_period_ms, size_t& c5gopen_period_orl );
  double get_c5gopen_period_in_usec( const size_t& c5gopen_period_orl );
  double get_c5gopen_period_in_nsec( const size_t& c5gopen_period_orl );
}

#endif