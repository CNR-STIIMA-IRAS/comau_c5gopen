
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

#ifndef C5GOPEN_MQTT_H
#define C5GOPEN_MQTT_H

#include <stdio.h>
#include <stdlib.h>
#include <algorithm>

#include <comau_c5gopen_lpc/mqtt.h> 
#include <comau_c5gopen_lpc/c5gopen_driver.h>


#define MAX_PAYLOAD_SIZE 1024
#define DEFAULT_KEEP_ALIVE 60

namespace c5gopen
{
  class C5GOpenMQTT : public cnr::MQTTClient 
  {
  private:
    uint8_t payload[MAX_PAYLOAD_SIZE]; 
    uint32_t payload_len = 0;
    std::string topic_name;

  public:
    C5GOpenMQTT (const char *id, const char *host, int port, std::shared_ptr<cnr_logger::TraceLogger>& logger):
                  MQTTClient(id, host, port, logger)
    {
      //payload_ptr = reinterpret_cast<uint8_t*>(&payload_d);
    }
 
    ~C5GOpenMQTT() { };

    bool publish( const std::shared_ptr<c5gopen::C5GOpenDriver>& c5gopen_driver );

    using MQTTClient::publish; // to make the method of base class visible again
  }; 

  inline bool C5GOpenMQTT::publish( const std::shared_ptr<c5gopen::C5GOpenDriver>& c5gopen_driver )
  {
    //if ( c5gopen_driver->getSystemInitialized() )
    if (1)
    {
      std::map<size_t,c5gopen::RobotJointStateArray> robot_joint_state_link_log_ = c5gopen_driver->getRobotJointStateLinkArray( );      
      for (std::map<size_t,c5gopen::RobotJointStateArray>::iterator it=robot_joint_state_link_log_.begin(); it!=robot_joint_state_link_log_.end(); it++ )
      {
        char arm[10]; 
        sprintf(arm,"%d",it->first);
        
        // Real joints positions
        payload_len = (uint32_t) sizeof(it->second.real_pos);
        memset( payload, 0, MAX_PAYLOAD_SIZE );
        for (size_t idx=0; idx<payload_len/sizeof(double); idx++)
          snprintf ( (char*)payload + sizeof(double) * idx , MAX_PAYLOAD_SIZE, "%f", it->second.real_pos[idx] );

        topic_name = "robot/arm" + std::string(arm) + "/real_joints_positions";
        publish(payload, payload_len, topic_name);

        // Real joints velocities
        payload_len = (uint32_t) sizeof(it->second.real_vel);
        memset( payload, 0, MAX_PAYLOAD_SIZE );
        for (size_t idx=0; idx<payload_len/sizeof(double); idx++)
          snprintf ( (char*)payload + sizeof(double) * idx , MAX_PAYLOAD_SIZE, "%f", it->second.real_vel[idx] );

        topic_name = "robot/arm" + std::string(arm) + "/real_joints_velocities";
        publish(payload, payload_len, topic_name);

        // Target joints positions
        payload_len = (uint32_t) sizeof(it->second.target_pos);
        memset( payload, 0, MAX_PAYLOAD_SIZE );
        for (size_t idx=0; idx<payload_len/sizeof(double); idx++)
          snprintf ( (char*)payload + sizeof(double) * idx , MAX_PAYLOAD_SIZE, "%f", it->second.target_pos[idx] );
        
        topic_name = "robot/arm" + std::string(arm) + "/target_joints_positions";
        publish(payload, payload_len, topic_name);        

        // Target joints velocities
        payload_len = (uint32_t) sizeof(it->second.target_vel);
        memset( payload, 0, MAX_PAYLOAD_SIZE );
        for (size_t idx=0; idx<payload_len/sizeof(double); idx++)
          snprintf ( (char*)payload + sizeof(double) * idx , MAX_PAYLOAD_SIZE, "%f", it->second.target_vel[idx] );
        
        topic_name = "robot/arm" + std::string(arm) + "/target_joints_velocities";
        publish(payload, payload_len, topic_name);

      }

      std::map<size_t,c5gopen::RobotCartStateArray> robot_cart_state_log_ = c5gopen_driver->getRobotCartStateArray( );      
      for ( std::map<size_t,c5gopen::RobotCartStateArray>::iterator it=robot_cart_state_log_.begin(); it!=robot_cart_state_log_.end(); it++ )
      {
        char arm[10]; 
        sprintf(arm,"%d",it->first);

        // Real Cartesian positions
        uint32_t payload_len_d = (uint32_t) sizeof(it->second.real_pos);
        uint32_t payload_len_c = (uint32_t) sizeof(it->second.config_flags_real);
        payload_len = (uint32_t)( payload_len_d + payload_len_c );

        memset( payload, 0, MAX_PAYLOAD_SIZE );
        for (size_t idx=0; idx<payload_len_d/sizeof(double); idx++)
          snprintf ( (char*)payload + sizeof(double) * idx , MAX_PAYLOAD_SIZE, "%f", it->second.real_pos[idx] );

        for (size_t idx=0; idx<payload_len_c/sizeof(char); idx++)
          snprintf ( (char*)payload + sizeof(double) * sizeof(it->second.real_pos) + 1 + sizeof(char) * idx , MAX_PAYLOAD_SIZE, "%s", (char*) it->second.config_flags_real[idx] );

        topic_name = "robot/arm" + std::string(arm) + "/real_cartesian_positions";
        publish( payload, payload_len, topic_name );

        // Target Cartesian positions
        payload_len_d = (uint32_t) sizeof(it->second.target_pos);
        payload_len_c = (uint32_t) sizeof(it->second.config_flags_target);
        payload_len = (uint32_t)( payload_len_d + payload_len_c );

        memset( payload, 0, MAX_PAYLOAD_SIZE );
        for (size_t idx=0; idx<payload_len_d/sizeof(double); idx++)
          snprintf ( (char*)payload + sizeof(double) * idx , MAX_PAYLOAD_SIZE, "%f", it->second.target_pos[idx] );

        for (size_t idx=0; idx<payload_len_c/sizeof(char); idx++)
          snprintf ( (char*)payload + sizeof(double) * sizeof(it->second.target_pos) + 1 + sizeof(char) * idx , MAX_PAYLOAD_SIZE, "%s", (char*) it->second.config_flags_target[idx] );
          
        topic_name = "robot/arm" + std::string(arm) + "/target_cartesian_positions";
        publish( payload, payload_len, topic_name );

      }

      std::map<size_t,c5gopen::RobotGenericArray> robot_motor_current_log_ = c5gopen_driver->getRobotMotorCurrentArray( );      
      for (  std::map<size_t,c5gopen::RobotGenericArray>::iterator it=robot_motor_current_log_.begin(); it!=robot_motor_current_log_.end(); it++ )
      {
        char arm[10]; 
        sprintf(arm,"%d",it->first);
        payload_len = (uint32_t) sizeof(it->second.value);

        // Motor currents
        memset( payload, 0, MAX_PAYLOAD_SIZE );
        for (size_t idx=0; idx<payload_len/sizeof(double); idx++)
          snprintf ( (char*)payload + sizeof(double) * idx , MAX_PAYLOAD_SIZE, "%f", it->second.value[idx] );

        topic_name = "robot/arm" + std::string(arm) + "/motor_currents";
        publish( payload, payload_len, topic_name );
      }

      return true;
    }
    else
    {
      std::cout << cnr_logger::RED() << "C5GOpen not initialized." << cnr_logger::RESET() << std::endl;
      return false;
    }
   
  }
} // end namespace

#endif //SIMPLECLIENT_MQTT_H
