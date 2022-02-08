
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
    C5GOpenMQTT (const char *id, const char *host, int port, const std::shared_ptr<cnr_logger::TraceLogger>& logger):
                  MQTTClient(id, host, port, logger)
    {
      // Nothing to do here  
    }
 
    ~C5GOpenMQTT() { };

    bool publishData( const std::shared_ptr<c5gopen::C5GOpenDriver>& c5gopen_driver );
    bool subscribeTopic( const std::string& sub_topic_name );
    bool updateRobotTargetTrajectory( const std::shared_ptr<c5gopen::C5GOpenDriver>& c5gopen_driver );
    
  }; 

  inline bool C5GOpenMQTT::publishData( const std::shared_ptr<c5gopen::C5GOpenDriver>& c5gopen_driver )
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

        memcpy((char*)(payload + sizeof(it->second.real_pos)), it->second.config_flags_real, sizeof(it->second.config_flags_real) );  


        topic_name = "robot/arm" + std::string(arm) + "/real_cartesian_positions";
        publish( payload, payload_len, topic_name );

        // Target Cartesian positions
        payload_len_d = (uint32_t) sizeof(it->second.target_pos);
        payload_len_c = (uint32_t) sizeof(it->second.config_flags_target);
        payload_len = (uint32_t)( payload_len_d + payload_len_c );

        memset( payload, 0, MAX_PAYLOAD_SIZE );
        for (size_t idx=0; idx<payload_len_d/sizeof(double); idx++)
          snprintf ( (char*)payload + sizeof(double) * idx , MAX_PAYLOAD_SIZE, "%f", it->second.target_pos[idx] );

        memcpy((char*)(payload + sizeof(it->second.target_pos)), it->second.config_flags_target, sizeof(it->second.config_flags_target) );

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
      CNR_ERROR( logger_, "C5GOpen not initialized.");
      return false;
    } 
  }


  inline bool C5GOpenMQTT::subscribeTopic( const std::string& sub_topic_name )
  {
    if ( subscribe( NULL, sub_topic_name.c_str(), 1 ) != MOSQ_ERR_SUCCESS )
      return false;

    CNR_INFO(logger_, "Subscribed topic: " << sub_topic_name.c_str() );

    return true;
  }

  inline bool C5GOpenMQTT::updateRobotTargetTrajectory( const std::shared_ptr<c5gopen::C5GOpenDriver>& c5gopen_driver )
  {
    if (loop() != MOSQ_ERR_SUCCESS)
      return false;
    
    std::map<std::string,std::pair<int, cnr::MQTTPayload>> last_messages = getLastReceivedMessage( );

    for (auto it=last_messages.begin(); it!=last_messages.end(); it++)
    {
      std::size_t found_arm = it->first.find("arm");
      if ( found_arm == std::string::npos )
        CNR_ERROR(logger_, "Cannot find 'arm' in the MQTT subscribed topic.");

      std::size_t found_delimiter = it->first.find("/",found_arm+1);
      if ( found_delimiter == std::string::npos )
        CNR_ERROR(logger_, "Cannot find delimiter '/' after 'arm' in the MQTT subscribed topic.");
      
      char arm_number[10] = {0};
      it->first.copy(arm_number,found_delimiter-(found_arm+3),found_arm+3);
    
      size_t arm = atoi(std::string(arm_number).c_str());

      std::pair<int, cnr::MQTTPayload> msg_complete = it->second;
      
      // Expected payload 8bytes every joints -> maximum joint expected is 10
      if( std::get<0>(msg_complete)%sizeof(double) != 0 || std::get<0>(msg_complete)/sizeof(double) > ORL_MAX_AXIS )
      {
        CNR_ERROR(logger_, "Invalid number of bytes for the subscribed topic: " << it->first 
                          << ". Received: " << std::get<0>(msg_complete) << " bytes.");  
        return false;
      }

      char c[8] = {0};
      c5gopen::RobotJointState target_joint_position;
      
      for(size_t idx=0; idx<std::get<0>(msg_complete)/sizeof(double); idx++)
      {
        memcpy(c, std::get<1>(msg_complete).payload + idx * sizeof(double), sizeof(double));
        target_joint_position.target_pos.value[idx] = atof(c);
        memset(c,0x0,sizeof(double));
      } 
      
      c5gopen_driver->setRobotJointAbsoluteTargetPosition(arm, target_joint_position); 
    }
     
    return true;
  }

} // end namespace

#endif //SIMPLECLIENT_MQTT_H
