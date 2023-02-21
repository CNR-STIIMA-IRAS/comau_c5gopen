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

  void C5GOpenMsgDecoder::on_message(const struct mosquitto_message *msg)
  {
    mtx_mqtt_.lock();    
    if (msg->payloadlen < MAX_PAYLOAD_SIZE)
    {
      char* buffer = new char[msg->payloadlen];
      memcpy(buffer, msg->payload, msg->payloadlen);

      std::string buffer_str(buffer);
    
      Json::Reader reader;
      Json::Value root;
    
      reader.parse(buffer_str,root);
      
      c5gopen::RobotJointState joint_positions;
      
      joint_positions.target_pos.value[0] = root["J1"].asDouble();
      joint_positions.target_pos.value[1] = root["J2"].asDouble();
      joint_positions.target_pos.value[2] = root["J3"].asDouble();
      joint_positions.target_pos.value[3] = root["J4"].asDouble();
      joint_positions.target_pos.value[4] = root["J5"].asDouble();
      joint_positions.target_pos.value[5] = root["J6"].asDouble();
      joint_positions.target_pos.value[6] = root["J7"].asDouble();
      joint_positions.target_pos.value[7] = root["J8"].asDouble();
      joint_positions.target_pos.value[8] = root["J9"].asDouble();
      joint_positions.target_pos.value[9] = root["J10"].asDouble();

      // The absolute trajectory need to be in degree
      joint_positions.target_pos.unit_type = ORL_POSITION_LINK_DEGREE;

      last_received_msg_[std::string(msg->topic)] = joint_positions;
    
      delete[] buffer;
    }
    
    mtx_mqtt_.unlock();
  }
    
  std::map<std::string,c5gopen::RobotJointState> C5GOpenMsgDecoder::getLastReceivedMessage()
  {
    return last_received_msg_;
  }

  C5GOpenMQTT::C5GOpenMQTT( const char *id, const char *host, int port,
                            const std::string& loop_timeout,
                            const std::shared_ptr<c5gopen::C5GOpenDriver>& c5gopen_driver 
                            const std::shared_ptr<cnr_logger::TraceLogger>& logger):
                            loop_timeout_(loop_timeout),
                            c5gopen_driver_(c5gopen_driver),
                            logger_(logger)
  {
    try
    {
      c5gopen_msg_decoder_ = new c5gopen::C5GOpenMsgDecoder();
      mqtt_client_ = new cnr::mqtt::MQTTClient(id, host, port, c5gopen_msg_decoder_, logger_);

      mqtt_thread_ = std::thread(&c5gopen::C5GOpenDriver::MQTTThread, this); 
      mqtt_thread_.detach();
    }
    catch(const std::exception& e)
    {
      std::cerr << e.what() << '\n';
    }
  }

  C5GOpenMQTT::~C5GOpenMQTT() 
  { 
    delete c5gopen_msg_decoder_;
    delete mqtt_client_;
  }

  void C5GOpenMQTT::MQTTThread()
  {
    mqtt_thread_status_ = thread_status::RUNNING;

    while ( c5gopen_driver_->getC5GOpenThreadsStatus() != c5gopen::thread_status::CLOSED ||
            c5gopen_driver_->getComThreadsStatus() != c5gopen::thread_status::CLOSED || 
            c5gopen_driver_->getLoopConsoleThreadsStatus() != c5gopen::thread_status::CLOSED )
    {
      if (!iot_client->publishData( ))
       CNR_WARN( logger, "Can't publish data to MQTT broker.");     

      if (!iot_client->updateRobotTargetTrajectory( ))
        CNR_WARN( logger, "Can't update robot target trajectory.");
      std::this_thread::sleep_for(std::chrono::microseconds(1));
    }

    mqtt_thread_status_ = thread_status::CLOSED;
    CNR_INFO( *logger_, "MQTT theread closed." );
  }

  thread_status C5GOpenMQTT::getMQTTThreadsStatus()
  {
    return mqtt_thread_status_;
  }


  bool C5GOpenMQTT::publishData( )
  {
    bool system_initialized = true;

#ifndef C5GOPEN_NOT_ENABLED
    system_initialized = c5gopen_driver_->getSystemInitialized();
#endif

    if ( system_initialized )
    {
      Clock::time_point start_time_pub = Clock::now();

      std::map<size_t,c5gopen::RobotJointStateArray> robot_joint_state_link_log_ = c5gopen_driver_->getRobotJointStateLinkArray( );      
   
      char arm[10];
    
      for ( std::map<size_t,c5gopen::RobotJointStateArray>::iterator it=robot_joint_state_link_log_.begin(); it!=robot_joint_state_link_log_.end(); it++ )
      {       
        Json::Value root;
        std::string json_file;
        char jnt_name[10];

        sprintf(arm,"%d",it->first);
       
        root["time"] = it->second.time_us;
        
        ///////////////////////////////////
        // Real joints positions

        topic_name_ = "robot/arm" + std::string(arm) + "/real_joints_positions";
        
        for (size_t idx=0; idx<ORL_MAX_AXIS; idx++)
        {
          sprintf(jnt_name,"J%d",idx+1); 
          root[jnt_name] =  it->second.real_pos[idx];
        }
                
        json_file = Json::FastWriter().write(root);
        payload_len_ = json_file.length() + 1;

        if (payload_len_ < MAX_PAYLOAD_SIZE)
        {
          strcpy((char *)payload_, json_file.c_str());
          
          if ( mqtt_client_->publish(payload_, payload_len_, topic_name_) != MOSQ_ERR_SUCCESS )
          {
            CNR_ERROR( logger_, "Error while publishing the topic " << topic_name_ );  
            return false;
          }
        }
        else
        {
          CNR_WARN( logger_, "Payload length exceed max limit for the topic " << topic_name_);    
        }
          
      
        ///////////////////////////////////
        // Real joints velocities

        topic_name_ = "robot/arm" + std::string(arm) + "/real_joints_velocities";
        
        for (size_t idx=0; idx<ORL_MAX_AXIS; idx++)
        {
          sprintf(jnt_name,"J%d",idx+1); 
          root[jnt_name] =  it->second.real_vel[idx];
        }
        
        json_file = Json::FastWriter().write(root);
        payload_len_ = json_file.length() + 1;

        if (payload_len_ < MAX_PAYLOAD_SIZE)
        {
          strcpy((char *)payload_, json_file.c_str());
          
          if ( mqtt_client_->publish(payload_, payload_len_, topic_name_) != MOSQ_ERR_SUCCESS )
          {
            CNR_ERROR( logger_, "Error while publishing the topic " << topic_name_ );  
            return false;
          }
        }
        else
        {
          CNR_WARN( logger_, "Payload length exceed max limit for the topic " << topic_name_);    
        }

     
        ///////////////////////////////////
        // Target joints positions

        topic_name_ = "robot/arm" + std::string(arm) + "/target_joints_positions";
        
        for (size_t idx=0; idx<ORL_MAX_AXIS; idx++)
        {
          sprintf(jnt_name,"J%d",idx+1); 
          root[jnt_name] =  it->second.target_pos[idx];
        }
                
        root["time"] = it->second.time_us;
        
        json_file = Json::FastWriter().write(root);
        payload_len_ = json_file.length() + 1;

        if (payload_len_ < MAX_PAYLOAD_SIZE)
        {
          strcpy((char *)payload_, json_file.c_str());
          
          if ( mqtt_client_->publish(payload_, payload_len_, topic_name_) != MOSQ_ERR_SUCCESS )
          {
            CNR_ERROR( logger_, "Error while publishing the topic " << topic_name_ );  
            return false;
          }
        }
        else
        {
          CNR_WARN( logger_, "Payload length exceed max limit for the topic " << topic_name_);    
        }

        ///////////////////////////////////
        // Target joints velocities

        topic_name_ = "robot/arm" + std::string(arm) + "/target_joints_velocities";
        
        for (size_t idx=0; idx<ORL_MAX_AXIS; idx++)
        {
          sprintf(jnt_name,"J%d",idx+1); 
          root[jnt_name] =  it->second.target_vel[idx];
        }
        
        json_file = Json::FastWriter().write(root);
        payload_len_ = json_file.length() + 1;

        if (payload_len_ < MAX_PAYLOAD_SIZE)
        {
          strcpy((char *)payload_, json_file.c_str());
          
          if ( mqtt_client_->publish(payload_, payload_len_, topic_name_) != MOSQ_ERR_SUCCESS )
          {
            CNR_ERROR( logger_, "Error while publishing the topic " << topic_name_ );  
            return false;
          }
        }
        else
        {
          CNR_WARN( logger_, "Payload length exceed max limit for the topic " << topic_name_);    
        }

      }

      std::map<size_t,c5gopen::RobotCartStateArray> robot_cart_state_log_ = c5gopen_driver_->getRobotCartStateArray( );      
      for ( std::map<size_t,c5gopen::RobotCartStateArray>::iterator it=robot_cart_state_log_.begin(); it!=robot_cart_state_log_.end(); it++ )
      {
        Json::Value root;
        std::string json_file;
        
        //char arm[10]; 
        sprintf(arm,"%d",it->first);

        root["time"] = it->second.time_us;

        ///////////////////////////////////  
        // Real Cartesian positions

        topic_name_ = "robot/arm" + std::string(arm) + "/real_cartesian_positions";
        
        root["x"] =  it->second.real_pos[0];
        root["y"] =  it->second.real_pos[1];
        root["z"] =  it->second.real_pos[2];
        root["a"] =  it->second.real_pos[3];
        root["e"] =  it->second.real_pos[4];
        root["r"] =  it->second.real_pos[5];
        root["config_flags"] =  it->second.config_flags_real;
        
        json_file = Json::FastWriter().write(root);
        payload_len_ = json_file.length() + 1;

        if (payload_len_ < MAX_PAYLOAD_SIZE)
        {
          strcpy((char *)payload_, json_file.c_str());
          
          if ( mqtt_client_->publish(payload_, payload_len_, topic_name_) != MOSQ_ERR_SUCCESS )
          {
            CNR_ERROR( logger_, "Error while publishing the topic " << topic_name_ );  
            return false;
          }
        }
        else
        {
          CNR_WARN( logger_, "Payload length exceed max limit for the topic " << topic_name_);    
        }


        ///////////////////////////////////
        // Target Cartesian positions

        topic_name_ = "robot/arm" + std::string(arm) + "/target_cartesian_positions";
        
        root["x"] =  it->second.target_pos[0];
        root["y"] =  it->second.target_pos[1];
        root["z"] =  it->second.target_pos[2];
        root["a"] =  it->second.target_pos[3];
        root["e"] =  it->second.target_pos[4];
        root["r"] =  it->second.target_pos[5];
        root["config_flags"] =  it->second.config_flags_target;
        
        json_file = Json::FastWriter().write(root);
        payload_len_ = json_file.length() + 1;

        if (payload_len_ < MAX_PAYLOAD_SIZE)
        {
          strcpy((char *)payload_, json_file.c_str());
          
          if ( mqtt_client_->publish(payload_, payload_len_, topic_name_) != MOSQ_ERR_SUCCESS )
          {
            CNR_ERROR( logger_, "Error while publishing the topic " << topic_name_ );  
            return false;
          }
        }
        else
        {
          CNR_WARN( logger_, "Payload length exceed max limit for the topic " << topic_name_);    
        }


      }

      std::map<size_t,c5gopen::RobotGenericArray> robot_motor_current_log_ = c5gopen_driver_->getRobotMotorCurrentArray( );      
      for (  std::map<size_t,c5gopen::RobotGenericArray>::iterator it=robot_motor_current_log_.begin(); it!=robot_motor_current_log_.end(); it++ )
      {
        Json::Value root;
        std::string json_file;
        char jnt_name[10];

        //char arm[10]; 
        sprintf(arm,"%d",it->first);

        root["time"] = it->second.time_us;

        ///////////////////////////////////
        // Motor currents

        topic_name_ = "robot/arm" + std::string(arm) + "/motor_currents";
        
        for (size_t idx=0; idx<ORL_MAX_AXIS; idx++)
        {
          sprintf(jnt_name,"J%d",idx+1); 
          root[jnt_name] =  it->second.value[idx];
        }
        
        json_file = Json::FastWriter().write(root);
        payload_len_ = json_file.length() + 1;

        if (payload_len_ < MAX_PAYLOAD_SIZE)
        {
          strcpy((char *)payload_, json_file.c_str());
          
          if ( mqtt_client_->publish(payload_, payload_len_, topic_name_) != MOSQ_ERR_SUCCESS )
          {
            CNR_ERROR( logger_, "Error while publishing the topic " << topic_name_ );  
            return false;
          }
        }
        else
        {
          CNR_WARN( logger_, "Payload length exceed max limit for the topic " << topic_name_);    
        }

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


  bool C5GOpenMQTT::subscribeTopic( const std::string& sub_topic_name  )
  {
    if ( mqtt_client_->subscribe( NULL, sub_topic_name.c_str(), 1 ) != MOSQ_ERR_SUCCESS )
      return false;

    CNR_INFO(logger_, "Subscribed topic: " << sub_topic_name.c_str() );

    return true;
  }

  bool C5GOpenMQTT::updateRobotTargetTrajectory( )
  {
    bool system_initialized = true;

#ifndef C5GOPEN_NOT_ENABLED
    system_initialized = c5gopen_driver_->getSystemInitialized();
#endif

    if ( system_initialized )
    {
      Clock::time_point start_time_sub = Clock::now();

      if (mqtt_client_->loop(loop_timeout_) != MOSQ_ERR_SUCCESS)
        return false;
      
      std::map<std::string,c5gopen::RobotJointState> last_messages = c5gopen_msg_decoder_->getLastReceivedMessage( );

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

        c5gopen_driver_->setRobotJointAbsoluteTargetPosition(arm, it->second); 
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