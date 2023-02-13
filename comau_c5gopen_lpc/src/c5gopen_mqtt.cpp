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

#include <comau_c5gopen_lpc/c5gopen_mqtt.h>

namespace c5gopen
{

  double tot_delta_pub_ = 0;
  int64_t tot_max_pub_ = 0;

  double tot_delta_sub_ = 0;
  int64_t tot_max_sub_ = 0;

  boost::circular_buffer<int64_t> delta_time_pub_(LOG_SAMPLES);
  boost::circular_buffer<int64_t> delta_time_sub_(LOG_SAMPLES);

  C5GOpenMQTT::C5GOpenMQTT( const char *id, const char *host, int port, const std::shared_ptr<cnr_logger::TraceLogger>& logger):
                            MQTTClient(id, host, port, logger)
  {
    // nothing to do here
  }

  C5GOpenMQTT::~C5GOpenMQTT() 
  { 
    // nothing to do here
  }

  bool C5GOpenMQTT::publishData( const std::shared_ptr<c5gopen::C5GOpenDriver>& c5gopen_driver )
  {
    bool system_initialized = true;

#ifndef C5GOPEN_NOT_ENABLED
    system_initialized = c5gopen_driver->getSystemInitialized();
#endif

    if ( system_initialized )
    {
      Clock::time_point start_time_pub = Clock::now();

      std::map<size_t,c5gopen::RobotJointStateArray> robot_joint_state_link_log_ = c5gopen_driver->getRobotJointStateLinkArray( );      
   
      for (std::map<size_t,c5gopen::RobotJointStateArray>::iterator it=robot_joint_state_link_log_.begin(); it!=robot_joint_state_link_log_.end(); it++ )
      {
        char arm[10]; 
        sprintf(arm,"%d",it->first);
       
        // Real joints positions
        uint32_t payload_len_d = (uint32_t) sizeof(it->second.real_pos);
        uint32_t payload_len_t = (uint32_t) sizeof(it->second.time_us)*2;
        payload_len_ = payload_len_d + payload_len_t;
        memset( payload_, 0, MAX_PAYLOAD_SIZE );
        snprintf( (char*)payload_, MAX_PAYLOAD_SIZE, "%lld", it->second.time_us );

        for (size_t idx=0; idx<payload_len_d/SIZE_OF_DOUBLE; idx++)
          snprintf ( (char*)payload_ + payload_len_t + SIZE_OF_DOUBLE * idx, MAX_PAYLOAD_SIZE, "%f", it->second.real_pos[idx] );

        topic_name_ = "robot/arm" + std::string(arm) + "/real_joints_positions";
        if ( publish(payload_, payload_len_, topic_name_) != MOSQ_ERR_SUCCESS )
          return false; 

        // Real joints velocities
        payload_len_d = (uint32_t) sizeof(it->second.real_vel);
        payload_len_ = payload_len_d + payload_len_t;
        memset( payload_, 0, MAX_PAYLOAD_SIZE );
        snprintf( (char*)payload_, MAX_PAYLOAD_SIZE, "%lld", it->second.time_us );

        for (size_t idx=0; idx<payload_len_d/SIZE_OF_DOUBLE; idx++)
          snprintf ( (char*)payload_ + payload_len_t + SIZE_OF_DOUBLE * idx , MAX_PAYLOAD_SIZE, "%f", it->second.real_vel[idx] );

        topic_name_ = "robot/arm" + std::string(arm) + "/real_joints_velocities";
        if ( publish(payload_, payload_len_, topic_name_) != MOSQ_ERR_SUCCESS )
          return false;

        // Target joints positions
        payload_len_d = (uint32_t) sizeof(it->second.target_pos);
        payload_len_ = payload_len_d + payload_len_t;
        memset( payload_, 0, MAX_PAYLOAD_SIZE );
        snprintf( (char*)payload_, MAX_PAYLOAD_SIZE, "%lld", it->second.time_us );
        
        for (size_t idx=0; idx<payload_len_d/SIZE_OF_DOUBLE; idx++)
          snprintf ( (char*)payload_ + payload_len_t + SIZE_OF_DOUBLE * idx , MAX_PAYLOAD_SIZE, "%f", it->second.target_pos[idx] );
        
        topic_name_ = "robot/arm" + std::string(arm) + "/target_joints_positions";
        if ( publish(payload_, payload_len_, topic_name_) != MOSQ_ERR_SUCCESS )
          return false;        

        // Target joints velocities
        payload_len_d = (uint32_t) sizeof(it->second.target_vel);
        payload_len_ = payload_len_d + payload_len_t;
        memset( payload_, 0, MAX_PAYLOAD_SIZE );
        snprintf( (char*)payload_, MAX_PAYLOAD_SIZE, "%lld", it->second.time_us );

        for (size_t idx=0; idx<payload_len_d/SIZE_OF_DOUBLE; idx++)
          snprintf ( (char*)payload_ + payload_len_t + SIZE_OF_DOUBLE * idx , MAX_PAYLOAD_SIZE, "%f", it->second.target_vel[idx] );
        
        topic_name_ = "robot/arm" + std::string(arm) + "/target_joints_velocities";
        if ( publish(payload_, payload_len_, topic_name_) != MOSQ_ERR_SUCCESS )
          return false;

      }

      std::map<size_t,c5gopen::RobotCartStateArray> robot_cart_state_log_ = c5gopen_driver->getRobotCartStateArray( );      
      for ( std::map<size_t,c5gopen::RobotCartStateArray>::iterator it=robot_cart_state_log_.begin(); it!=robot_cart_state_log_.end(); it++ )
      {
        char arm[10]; 
        sprintf(arm,"%d",it->first);

        // Real Cartesian positions
        uint32_t payload_len_t = (uint32_t) sizeof(it->second.time_us)*2;
        uint32_t payload_len_d = (uint32_t) sizeof(it->second.real_pos);
        uint32_t payload_len_c = (uint32_t) sizeof(it->second.config_flags_real);
        payload_len_ = payload_len_t + payload_len_d + payload_len_c;

        memset( payload_, 0, MAX_PAYLOAD_SIZE );
        snprintf( (char*)payload_, MAX_PAYLOAD_SIZE, "%lld", it->second.time_us );

        for (size_t idx=0; idx<payload_len_d/SIZE_OF_DOUBLE; idx++)
          snprintf ( (char*)payload_ + payload_len_t + SIZE_OF_DOUBLE * idx , MAX_PAYLOAD_SIZE, "%f", it->second.real_pos[idx] );

        memcpy((char*)(payload_ + payload_len_t + sizeof(it->second.real_pos)), it->second.config_flags_real, sizeof(it->second.config_flags_real) );  


        topic_name_ = "robot/arm" + std::string(arm) + "/real_cartesian_positions";
        if ( publish( payload_, payload_len_, topic_name_ ) != MOSQ_ERR_SUCCESS )
          return false;

        // Target Cartesian positions
        payload_len_d = (uint32_t) sizeof(it->second.target_pos);
        payload_len_c = (uint32_t) sizeof(it->second.config_flags_target);
        payload_len_ = payload_len_t + payload_len_d + payload_len_c;

        memset( payload_, 0, MAX_PAYLOAD_SIZE );
        snprintf( (char*)payload_, MAX_PAYLOAD_SIZE, "%lld", it->second.time_us );

        for (size_t idx=0; idx<payload_len_d/SIZE_OF_DOUBLE; idx++)
          snprintf ( (char*)payload_ + payload_len_t + SIZE_OF_DOUBLE * idx , MAX_PAYLOAD_SIZE, "%f", it->second.target_pos[idx] );

        memcpy((char*)(payload_ + payload_len_t + sizeof(it->second.target_pos)), it->second.config_flags_target, sizeof(it->second.config_flags_target) );

        topic_name_ = "robot/arm" + std::string(arm) + "/target_cartesian_positions";
        if ( publish( payload_, payload_len_, topic_name_ ) != MOSQ_ERR_SUCCESS )
          return false;
      }

      std::map<size_t,c5gopen::RobotGenericArray> robot_motor_current_log_ = c5gopen_driver->getRobotMotorCurrentArray( );      
      for (  std::map<size_t,c5gopen::RobotGenericArray>::iterator it=robot_motor_current_log_.begin(); it!=robot_motor_current_log_.end(); it++ )
      {
        char arm[10]; 
        sprintf(arm,"%d",it->first);
        uint32_t payload_len_t = (uint32_t) sizeof(it->second.time_us)*2;
        uint32_t payload_len_d = (uint32_t) sizeof(it->second.value); 
        payload_len_ = payload_len_t + payload_len_d;

        // Motor currents
        memset( payload_, 0, MAX_PAYLOAD_SIZE );
        snprintf( (char*)payload_, MAX_PAYLOAD_SIZE, "%lld", it->second.time_us );

        for (size_t idx=0; idx<payload_len_/SIZE_OF_DOUBLE; idx++)
          snprintf ( (char*)payload_ + payload_len_t + SIZE_OF_DOUBLE * idx , MAX_PAYLOAD_SIZE, "%f", it->second.value[idx] );

        topic_name_ = "robot/arm" + std::string(arm) + "/motor_currents";
        if ( publish( payload_, payload_len_, topic_name_ ) != MOSQ_ERR_SUCCESS )
          return false;
      }

      static int cycle_pub = 0;
      static bool buff_full = false;

      Clock::time_point now = Clock::now();
      microsec us = std::chrono::duration_cast<microsec>(now - start_time_pub);

      if( us.count() > tot_max_pub_ )  
        tot_max_pub_ = us.count();
    
      delta_time_pub_.push_back(us.count());
      
      int64_t cum_delta = 0;
      for (boost::circular_buffer<int64_t>::const_iterator it = delta_time_pub_.begin(); it != delta_time_pub_.end(); it++)
        cum_delta += *it;
        
      cycle_pub++;

      if ( cycle_pub < LOG_SAMPLES && !buff_full )
        tot_delta_pub_ = (double) (cum_delta / cycle_pub);  
      else
      {
        tot_delta_pub_ = (double) (cum_delta / LOG_SAMPLES);
        buff_full = true;
      }

      if( cycle_pub % 100000 == 0)
        CNR_DEBUG(logger_, "C5GOpen MQTT publish stats [ average / max ] us : [ " << tot_delta_pub_ << "/ " << tot_max_pub_ << " ] us" );

    }
    else
      CNR_WARN( logger_, "C5GOpen not initialized yet, can't publish C5GOpen data.");

    return true;
  }


  bool C5GOpenMQTT::subscribeTopic( const std::string& sub_topic_name )
  {
    if ( subscribe( NULL, sub_topic_name.c_str(), 1 ) != MOSQ_ERR_SUCCESS )
      return false;

    CNR_INFO(logger_, "Subscribed topic: " << sub_topic_name.c_str() );

    return true;
  }

  bool C5GOpenMQTT::updateRobotTargetTrajectory( const std::shared_ptr<c5gopen::C5GOpenDriver>& c5gopen_driver, const size_t& loop_timeout )
  {
    bool system_initialized = true;

#ifndef C5GOPEN_NOT_ENABLED
    system_initialized = c5gopen_driver->getSystemInitialized();
#endif

    if ( system_initialized )
    {
      Clock::time_point start_time_sub = Clock::now();

      if (loop(loop_timeout) != MOSQ_ERR_SUCCESS)
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
        if( std::get<0>(msg_complete)%SIZE_OF_DOUBLE != 0 || std::get<0>(msg_complete)/SIZE_OF_DOUBLE > ORL_MAX_AXIS )
        {
          CNR_ERROR(logger_, "Invalid number of bytes for the subscribed topic: " << it->first 
                            << ". Received: " << std::get<0>(msg_complete) << " bytes.");  
          return false;
        }

        char c[8] = {0};
        c5gopen::RobotJointState target_joint_position;
        
        for(size_t idx=0; idx<std::get<0>(msg_complete)/SIZE_OF_DOUBLE; idx++)
        {
          memcpy(c, std::get<1>(msg_complete).payload + idx * SIZE_OF_DOUBLE, SIZE_OF_DOUBLE);
          target_joint_position.target_pos.value[idx] = atof(c);
          memset(c,0x0,SIZE_OF_DOUBLE);
        } 

        c5gopen_driver->setRobotJointAbsoluteTargetPosition(arm, target_joint_position); 
      }
      
      static bool buff_full = false;
      static int cycle_sub = 0;

      Clock::time_point now = Clock::now();
      microsec us = std::chrono::duration_cast<microsec>(now - start_time_sub);

      if( us.count() > tot_max_sub_ )  
        tot_max_sub_ = us.count();

      delta_time_sub_.push_back(us.count());
      
      int64_t cum_delta = 0;
      for (boost::circular_buffer<int64_t>::const_iterator it = delta_time_sub_.begin(); it != delta_time_sub_.end(); it++)
        cum_delta += *it;

      cycle_sub++;

      if ( cycle_sub < LOG_SAMPLES && !buff_full )
        tot_delta_sub_ = cum_delta / cycle_sub;  
      else
      {
        tot_delta_sub_ = cum_delta / LOG_SAMPLES;
        buff_full = true;
      }
      
      if( cycle_sub % 100000 == 0)
        CNR_DEBUG(logger_, "C5GOpen MQTT subscribe stats [ average / max ] us : [ " << tot_delta_sub_ << "/ " << tot_max_sub_ << " ] us" );
    }

    return true;
  }


} // end namespace c5gopen