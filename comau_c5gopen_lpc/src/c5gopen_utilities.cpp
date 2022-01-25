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
#include <fstream>

#include <cnr_logger/cnr_logger_macros.h>

#include <comau_c5gopen_lpc/c5gopen_utilities.h>

namespace c5gopen
{

  bool load_c5gopen_parameters( const std::string& config_file_name, C5GOpenCfg& c5gopen_cfg )
  {
    // Load parameters form YAML file
    std::ifstream if1(config_file_name.c_str());
    if (!if1.good())
    {
      std::cout << cnr_logger::RED() << "Error: the configuration file: " << config_file_name << " does not exists." << cnr_logger::RESET() << std::endl;
      return false;
    }  
      
    YAML::Node cfg_file = YAML::LoadFile(config_file_name);

    // Extract parameters from configuration file

    // Load C5GOpen controller parameters
    if ( !( cfg_file["c5gopen_ctrl"]["ctrl_idx"] && cfg_file["c5gopen_ctrl"]["ctrl_idx"].IsScalar() ))
    {
      std::cout << cnr_logger::RED() << "Error: wrong format or missing element 'c5gopen_ctrl' -> 'ctrl_idx' " << cnr_logger::RESET() << std::endl;
      return false;
    }
    else
      c5gopen_cfg.ctrl_idx_ = get_orl_ctrl_num(cfg_file["c5gopen_ctrl"]["ctrl_idx"].as<size_t>());
    

    if ( !( cfg_file["c5gopen_ctrl"]["ip_ctrl"] && cfg_file["c5gopen_ctrl"]["ip_ctrl"].IsScalar() ) )
    {
      std::cout << cnr_logger::RED() << "Error: wrong format or missing element 'c5gopen_ctrl' -> 'ip_ctrl' " << cnr_logger::RESET() << std::endl;
      return false;
    }
    else
      c5gopen_cfg.ip_ctrl_ = cfg_file["c5gopen_ctrl"]["ip_ctrl"].as<std::string>();   


    if ( !( cfg_file["c5gopen_ctrl"]["sys_id"] && cfg_file["c5gopen_ctrl"]["sys_id"].IsScalar() ) )
    {
      std::cout << cnr_logger::RED() << "Error: wrong format or missing element 'c5gopen_ctrl' -> 'sys_id' " << cnr_logger::RESET() << std::endl;
      return false;
    }
    else
      c5gopen_cfg.sys_id_ = cfg_file["c5gopen_ctrl"]["sys_id"].as<std::string>();
    

    if ( !(cfg_file["c5gopen_ctrl"]["active_arms"] && cfg_file["c5gopen_ctrl"]["active_arms"].IsSequence() ) )
    {
      std::cout << cnr_logger::RED() << "Error: wrong format or missing element 'c5gopen_ctrl' -> 'active_arms' " << cnr_logger::RESET() << std::endl;
      return false;
    }
    else
      c5gopen_cfg.active_arms_ = cfg_file["c5gopen_ctrl"]["active_arms"].as<std::vector<size_t>>();
     
    c5gopen_cfg.max_number_of_arms_ = c5gopen_cfg.active_arms_.size();
    

    if ( !(cfg_file["c5gopen_ctrl"]["period"] && cfg_file["c5gopen_ctrl"]["period"].IsScalar() ))
    {
      std::cout << cnr_logger::RED() << "Error: wrong format or missing element 'c5gopen_ctrl' -> 'period' " << cnr_logger::RESET() << std::endl;
      return false;
    }
    else
    {
      if (!decode_c5gopen_frequency( cfg_file["c5gopen_ctrl"]["period"].as<double>(), c5gopen_cfg.c5gopen_period_orl_ ) )
        return false;
    }

    // Load robot frames
    if ( !(cfg_file["robot_frames"]["base_frame"] && cfg_file["robot_frames"]["base_frame"].IsSequence() ))
    {
      std::cout << cnr_logger::RED() << "Error: wrong format or missing element 'robot_frames' -> 'base_frame' " << cnr_logger::RESET() << std::endl;
      return false;
    }
    else
    {
      if ( !set_frames( cfg_file["robot_frames"]["base_frame"].as<std::vector<double>>(), c5gopen_cfg.base_frame_ ) )
        return false;
    }

    if ( !(cfg_file["robot_frames"]["user_frame"] && cfg_file["robot_frames"]["user_frame"].IsSequence() ))
    {
      std::cout << cnr_logger::RED() << "Error: wrong format or missing element 'robot_frames' -> 'user_frame' " << cnr_logger::RESET() << std::endl;
      return false;
    }
    else
    {
      if ( !set_frames( cfg_file["robot_frames"]["user_frame"].as<std::vector<double>>(), c5gopen_cfg.user_frame_ ) )
        return false;
    }

    if ( !(cfg_file["robot_frames"]["tool_frame"] && cfg_file["robot_frames"]["tool_frame"].IsSequence() ))
    {
      std::cout << cnr_logger::RED() << "Error: wrong format or missing element 'robot_frames' -> 'tool_frame' " << cnr_logger::RESET() << std::endl;
      return false;
    }
    else
    {    
      if ( !set_frames( cfg_file["robot_frames"]["tool_frame"].as<std::vector<double>>(), c5gopen_cfg.tool_frame_ ) )
        return false;
    }


    // Load MQTT configuration parameters
    if ( !(cfg_file["mqtt"]["client_id"] && cfg_file["mqtt"]["client_id"].IsScalar()) )
    {
      std::cout << cnr_logger::RED() << "Error: wrong format or missing element 'mqtt' -> 'client_id' " << cnr_logger::RESET() << std::endl;
      return false;
    }
    else
      c5gopen_cfg.mqtt_client_id_ = cfg_file["mqtt"]["client_id"].as<std::string>();
    

    if ( !(cfg_file["mqtt"]["broker_address"] && cfg_file["mqtt"]["broker_address"].IsScalar()) )
    {
      std::cout << cnr_logger::RED() << "Error: wrong format or missing element 'mqtt' -> 'broker_address' " << cnr_logger::RESET() << std::endl;
      return false;
    }
    else
      c5gopen_cfg.mqtt_broker_address_ = cfg_file["mqtt"]["broker_address"].as<std::string>();
    

    if ( !(cfg_file["mqtt"]["mqtt_port"] && cfg_file["mqtt"]["mqtt_port"].IsScalar()) )
    {
      std::cout << cnr_logger::RED() << "Error: wrong format or missing element 'mqtt' -> 'mqtt_port' " << cnr_logger::RESET() << std::endl;
      return false;
    }
    else
      c5gopen_cfg.mqtt_port_ = cfg_file["mqtt"]["mqtt_port"].as<std::string>();
    

    if ( !(cfg_file["mqtt"]["mqtt_topic"] && cfg_file["mqtt"]["mqtt_topic"].IsScalar()) )
    {
      std::cout << cnr_logger::RED() << "Error: wrong format or missing element 'mqtt' -> 'mqtt_topic' " << cnr_logger::RESET() << std::endl;
      return false;
    }
    else
      c5gopen_cfg.mqtt_topic_ = cfg_file["mqtt"]["mqtt_topic"].as<std::string>();


    // Load cnr_logger configuration parameters
    if ( !(cfg_file["cnr_logger"]["logger_cfg_file"] && cfg_file["cnr_logger"]["logger_cfg_file"].IsScalar()) )
    {
      std::cout << cnr_logger::RED() << "Error: wrong format or missing element 'cnr_logger' -> 'logger_cfg_file' " << cnr_logger::RESET() << std::endl;
      return false;
    }
    else
    {
      c5gopen_cfg.cnr_logger_cfg_file  = cfg_file["cnr_logger"]["logger_cfg_file"].as<std::string>();
      std::ifstream if2(c5gopen_cfg.cnr_logger_cfg_file.c_str());
      if (!if2.good())
      {
        std::cout << cnr_logger::RED() << "Error: the CNR_LOGGER configuration file: " << c5gopen_cfg.cnr_logger_cfg_file << " does not exists." << cnr_logger::RESET() << std::endl;
        return false;
      }  
    }
      
    std::cout << cnr_logger::WHITE() << "Configuration parameter loaded from file: " << config_file_name << cnr_logger::RESET() << std::endl;

    return true;
  }

  size_t get_orl_ctrl_num( const size_t& ctrl_idx )
  {
    if (ctrl_idx > 0 && ctrl_idx <= 32)
      return ctrl_idx-1;
    else
    {
      std::cout << cnr_logger::RED() << "Error: invalid controller index: values allowed are from 1 to 32, provided " << ctrl_idx << cnr_logger::RESET() << std::endl;
      return 1000;
    }
  }

  size_t get_orl_arm_num( const size_t& arm_idx )
  {     
    if (arm_idx > 0 && arm_idx <= 4)
      return arm_idx-1;
    else
    {
      std::cout << cnr_logger::RED() << "Error: arm index: values allowed are from 1 to 32, provided" << arm_idx << cnr_logger::RESET() << std::endl;
      return 1000;
    }
  }

  bool set_frames( const std::vector<double>& frame, ORL_cartesian_position& orl_frame )
  {     
    if (frame.size() == 6)
    {
      orl_frame.unit_type = ORL_CART_POSITION;
      orl_frame.x = frame.at(0);
      orl_frame.y = frame.at(1);
      orl_frame.z = frame.at(2);
      orl_frame.a = frame.at(3);
      orl_frame.e = frame.at(4);
      orl_frame.r = frame.at(5);
    }          
    else
    {
      std::cout << cnr_logger::RED() << "Error: frame wrong data size." << cnr_logger::RESET() << std::endl;
      return false;
    }

    return true;
  }

  bool decode_modality( const int& si_modality, std::string& string )
  {
    switch(si_modality)
    {
    case CRCOPEN_LISTEN:
      string = "CRCOPEN_LISTEN";
      break;
    case CRCOPEN_POS_ABSOLUTE:
      string = "CRCOPEN_POS_ABSOLUTE";
      break;
    case CRCOPEN_POS_RELATIVE:
      string = "CRCOPEN_POS_RELATIVE";
      break;
    case CRCOPEN_POS_ADDITIVE:
      string = "CRCOPEN_POS_ADDITIVE";
      break;
    case CRCOPEN_POS_ADDITIVE_SB:
      string = "CRCOPEN_POS_ADDITIVE_SB";
      break;
    case CRCOPEN_POS_ADDITIVE_SBE:
      string = "CRCOPEN_POS_ADDITIVE_SBE";
      break;
    default:
      string = "--"; 
      return false;
    }

    return true;
  }

  bool decode_c5gopen_frequency( const double& c5gopen_period_ms, size_t& c5gopen_period_orl )
  {
    if ( c5gopen_period_ms == 0.4 )
      c5gopen_period_orl = size_t(ORL_0_4_MILLIS);
    else if ( c5gopen_period_ms == 2.0 )
      c5gopen_period_orl = size_t(ORL_2_0_MILLIS);
    else if( c5gopen_period_ms == 4.0 )
      c5gopen_period_orl = size_t(ORL_4_0_MILLIS);
    else if( c5gopen_period_ms == 8.0 )
      c5gopen_period_orl = size_t(ORL_8_0_MILLIS);
    else if( c5gopen_period_ms == 16.0 )
      c5gopen_period_orl = size_t(ORL_16_0_MILLIS);
    else 
    {
      std::cout << cnr_logger::RED() << "Invalid working frequency! Allowed value are: 0.4 - 2.0 - 4.0 - 8.0 - 16.0 [ms]" << cnr_logger::RESET() << std::endl;
      return false;
    }

    return true;
  }

  double get_c5gopen_period_in_usec( const size_t& c5gopen_period_orl )
  {
    switch(c5gopen_period_orl)
    {
    case size_t(ORL_0_4_MILLIS):
      return 400.0;
    case size_t(ORL_2_0_MILLIS):
      return 2000.0;
    case size_t(ORL_4_0_MILLIS):
      return 4000.0;
    case size_t(ORL_8_0_MILLIS):
      return 8000.0;
    case size_t(ORL_16_0_MILLIS):
      return 16000.0;
    default:
      std::cout << cnr_logger::RED() << "Invalid working frequency! Allowed value are: 0.4 - 2.0 - 4.0 - 8.0 - 16.0 [ms]" << cnr_logger::RESET() << std::endl;
      return -1;
    }
  }

  double get_c5gopen_period_in_nsec( const size_t& c5gopen_period_orl)
  {
    return get_c5gopen_period_in_usec(c5gopen_period_orl) * pow(10.0,3.0);
  }

} // end namespace c5gopen