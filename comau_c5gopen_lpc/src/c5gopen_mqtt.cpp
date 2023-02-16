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

  C5GOpenMQTT::C5GOpenMQTT( const char *id, const char *host, int port, MsgDecoder* msg_decoder, 
                            const std::shared_ptr<cnr_logger::TraceLogger>& logger):
                            MQTTClient(id, host, port, logger)
  {
    if (msg_decoder == NULL)
    {
      std::cout << "NULL ptr to decoder pointer" << std::endl;
      return;
    }

    if (cnr::mqtt::init_library( msg_decoder ) < 0)
    {
      std::cout << "Cannot initialize the encoder and decoder library" << std::endl;
      return;
    }

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
   
      char arm[10];
    
      for ( std::map<size_t,c5gopen::RobotJointStateArray>::iterator it=robot_joint_state_link_log_.begin(); it!=robot_joint_state_link_log_.end(); it++ )
      {       
        Json::Value root;
        Json::StreamWriterBuilder builder;
        std::string json_file;
        char jnt_name[10];

        sprintf(arm,"%d",it->first);
       
        root["time"] = it->second.time_us;
        
        ///////////////////////////////////
        // Real joints positions
        // uint32_t payload_len_d = (uint32_t) sizeof(it->second.real_pos);
        // uint32_t payload_len_t = (uint32_t) sizeof(it->second.time_us)*2;
        // payload_len_ = payload_len_d + payload_len_t;
        // memset( payload_, 0, MAX_PAYLOAD_SIZE );
        // snprintf( (char*)payload_, MAX_PAYLOAD_SIZE, "%lld", it->second.time_us );

        // for (size_t idx=0; idx<payload_len_d/SIZE_OF_DOUBLE; idx++)
        //   snprintf ( (char*)payload_ + payload_len_t + SIZE_OF_DOUBLE * idx, MAX_PAYLOAD_SIZE, "%f", it->second.real_pos[idx] );
        
        // topic_name_ = "robot/arm" + std::string(arm) + "/real_joints_positions";
        // if ( publish(payload_, payload_len_, topic_name_) != MOSQ_ERR_SUCCESS )
        //   return false; 

        topic_name_ = "robot/arm" + std::string(arm) + "/real_joints_positions";
        
        for (size_t idx=0; idx<ORL_MAX_AXIS; idx++)
        {
          sprintf(jnt_name,"J%d",idx+1); 
          root[jnt_name] =  it->second.real_pos[idx];
        }
                
        json_file = Json::writeString(builder, root);
        payload_len_ = json_file.length() + 1;

        if (payload_len_ < MAX_PAYLOAD_SIZE)
        {
          strcpy(payload_, json_file.c_str());
          
          if ( publish(payload_, payload_len_, topic_name_) != MOSQ_ERR_SUCCESS )
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
        // payload_len_d = (uint32_t) sizeof(it->second.real_vel);
        // payload_len_ = payload_len_d + payload_len_t;
        // memset( payload_, 0, MAX_PAYLOAD_SIZE );
        // snprintf( (char*)payload_, MAX_PAYLOAD_SIZE, "%lld", it->second.time_us );

        // for (size_t idx=0; idx<payload_len_d/SIZE_OF_DOUBLE; idx++)
        //   snprintf ( (char*)payload_ + payload_len_t + SIZE_OF_DOUBLE * idx , MAX_PAYLOAD_SIZE, "%f", it->second.real_vel[idx] );

        // topic_name_ = "robot/arm" + std::string(arm) + "/real_joints_velocities";
        // if ( publish(payload_, payload_len_, topic_name_) != MOSQ_ERR_SUCCESS )
        //   return false;

        topic_name_ = "robot/arm" + std::string(arm) + "/real_joints_velocities";
        
        for (size_t idx=0; idx<ORL_MAX_AXIS; idx++)
        {
          sprintf(jnt_name,"J%d",idx+1); 
          root[jnt_name] =  it->second.real_vel[idx];
        }
        
        json_file = Json::writeString(builder, root);
        payload_len_ = json_file.length() + 1;

        if (payload_len_ < MAX_PAYLOAD_SIZE)
        {
          strcpy(payload_, json_file.c_str());
          
          if ( publish(payload_, payload_len_, topic_name_) != MOSQ_ERR_SUCCESS )
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
        // payload_len_d = (uint32_t) sizeof(it->second.target_pos);
        // payload_len_ = payload_len_d + payload_len_t;
        // memset( payload_, 0, MAX_PAYLOAD_SIZE );
        // snprintf( (char*)payload_, MAX_PAYLOAD_SIZE, "%lld", it->second.time_us );
        
        // for (size_t idx=0; idx<payload_len_d/SIZE_OF_DOUBLE; idx++)
        //   snprintf ( (char*)payload_ + payload_len_t + SIZE_OF_DOUBLE * idx , MAX_PAYLOAD_SIZE, "%f", it->second.target_pos[idx] );
        
        // topic_name_ = "robot/arm" + std::string(arm) + "/target_joints_positions";
        // if ( publish(payload_, payload_len_, topic_name_) != MOSQ_ERR_SUCCESS )
        //   return false;        

        topic_name_ = "robot/arm" + std::string(arm) + "/target_joints_positions";
        
        for (size_t idx=0; idx<ORL_MAX_AXIS; idx++)
        {
          sprintf(jnt_name,"J%d",idx+1); 
          root[jnt_name] =  it->second.target_pos[idx];
        }
                
        root["time"] = it->second.time_us;
        
        json_file = Json::writeString(builder, root);
        payload_len_ = json_file.length() + 1;

        if (payload_len_ < MAX_PAYLOAD_SIZE)
        {
          strcpy(payload_, json_file.c_str());
          
          if ( publish(payload_, payload_len_, topic_name_) != MOSQ_ERR_SUCCESS )
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
        // payload_len_d = (uint32_t) sizeof(it->second.target_vel);
        // payload_len_ = payload_len_d + payload_len_t;
        // memset( payload_, 0, MAX_PAYLOAD_SIZE );
        // snprintf( (char*)payload_, MAX_PAYLOAD_SIZE, "%lld", it->second.time_us );

        // for (size_t idx=0; idx<payload_len_d/SIZE_OF_DOUBLE; idx++)
        //   snprintf ( (char*)payload_ + payload_len_t + SIZE_OF_DOUBLE * idx , MAX_PAYLOAD_SIZE, "%f", it->second.target_vel[idx] );
        
        // topic_name_ = "robot/arm" + std::string(arm) + "/target_joints_velocities";
        // if ( publish(payload_, payload_len_, topic_name_) != MOSQ_ERR_SUCCESS )
        //   return false;

        topic_name_ = "robot/arm" + std::string(arm) + "/target_joints_velocities";
        
        for (size_t idx=0; idx<ORL_MAX_AXIS; idx++)
        {
          sprintf(jnt_name,"J%d",idx+1); 
          root[jnt_name] =  it->second.target_vel[idx];
        }
        
        json_file = Json::writeString(builder, root);
        payload_len_ = json_file.length() + 1;

        if (payload_len_ < MAX_PAYLOAD_SIZE)
        {
          strcpy(payload_, json_file.c_str());
          
          if ( publish(payload_, payload_len_, topic_name_) != MOSQ_ERR_SUCCESS )
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

      std::map<size_t,c5gopen::RobotCartStateArray> robot_cart_state_log_ = c5gopen_driver->getRobotCartStateArray( );      
      for ( std::map<size_t,c5gopen::RobotCartStateArray>::iterator it=robot_cart_state_log_.begin(); it!=robot_cart_state_log_.end(); it++ )
      {
        Json::Value root;
        Json::StreamWriterBuilder builder;
        std::string json_file;
        
        //char arm[10]; 
        sprintf(arm,"%d",it->first);

        root["time"] = it->second.time_us;

        ///////////////////////////////////  
        // Real Cartesian positions
        // uint32_t payload_len_t = (uint32_t) sizeof(it->second.time_us)*2;
        // uint32_t payload_len_d = (uint32_t) sizeof(it->second.real_pos);
        // uint32_t payload_len_c = (uint32_t) sizeof(it->second.config_flags_real);
        // payload_len_ = payload_len_t + payload_len_d + payload_len_c;

        // memset( payload_, 0, MAX_PAYLOAD_SIZE );
        // snprintf( (char*)payload_, MAX_PAYLOAD_SIZE, "%lld", it->second.time_us );

        // for (size_t idx=0; idx<payload_len_d/SIZE_OF_DOUBLE; idx++)
        //   snprintf ( (char*)payload_ + payload_len_t + SIZE_OF_DOUBLE * idx , MAX_PAYLOAD_SIZE, "%f", it->second.real_pos[idx] );

        // memcpy((char*)(payload_ + payload_len_t + sizeof(it->second.real_pos)), it->second.config_flags_real, sizeof(it->second.config_flags_real) );  

        // topic_name_ = "robot/arm" + std::string(arm) + "/real_cartesian_positions";
        // if ( publish( payload_, payload_len_, topic_name_ ) != MOSQ_ERR_SUCCESS )
        //   return false;

        topic_name_ = "robot/arm" + std::string(arm) + "/real_cartesian_positions";
        
        root["x"] =  it->second.real_pos[0];
        root["y"] =  it->second.real_pos[1];
        root["z"] =  it->second.real_pos[2];
        root["a"] =  it->second.real_pos[3];
        root["e"] =  it->second.real_pos[4];
        root["r"] =  it->second.real_pos[5];
        root["config_flags"] =  it->second.config_flags_real;
        
        json_file = Json::writeString(builder, root);
        payload_len_ = json_file.length() + 1;

        if (payload_len_ < MAX_PAYLOAD_SIZE)
        {
          strcpy(payload_, json_file.c_str());
          
          if ( publish(payload_, payload_len_, topic_name_) != MOSQ_ERR_SUCCESS )
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
        // payload_len_d = (uint32_t) sizeof(it->second.target_pos);
        // payload_len_c = (uint32_t) sizeof(it->second.config_flags_target);
        // payload_len_ = payload_len_t + payload_len_d + payload_len_c;

        // memset( payload_, 0, MAX_PAYLOAD_SIZE );
        // snprintf( (char*)payload_, MAX_PAYLOAD_SIZE, "%lld", it->second.time_us );

        // for (size_t idx=0; idx<payload_len_d/SIZE_OF_DOUBLE; idx++)
        //   snprintf ( (char*)payload_ + payload_len_t + SIZE_OF_DOUBLE * idx , MAX_PAYLOAD_SIZE, "%f", it->second.target_pos[idx] );

        // memcpy((char*)(payload_ + payload_len_t + sizeof(it->second.target_pos)), it->second.config_flags_target, sizeof(it->second.config_flags_target) );

        // topic_name_ = "robot/arm" + std::string(arm) + "/target_cartesian_positions";
        // if ( publish( payload_, payload_len_, topic_name_ ) != MOSQ_ERR_SUCCESS )
        //   return false;

        topic_name_ = "robot/arm" + std::string(arm) + "/target_cartesian_positions";
        
        root["x"] =  it->second.target_pos[0];
        root["y"] =  it->second.target_pos[1];
        root["z"] =  it->second.target_pos[2];
        root["a"] =  it->second.target_pos[3];
        root["e"] =  it->second.target_pos[4];
        root["r"] =  it->second.target_pos[5];
        root["config_flags"] =  it->second.config_flags_target;
        
        json_file = Json::writeString(builder, root);
        payload_len_ = json_file.length() + 1;

        if (payload_len_ < MAX_PAYLOAD_SIZE)
        {
          strcpy(payload_, json_file.c_str());
          
          if ( publish(payload_, payload_len_, topic_name_) != MOSQ_ERR_SUCCESS )
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

      std::map<size_t,c5gopen::RobotGenericArray> robot_motor_current_log_ = c5gopen_driver->getRobotMotorCurrentArray( );      
      for (  std::map<size_t,c5gopen::RobotGenericArray>::iterator it=robot_motor_current_log_.begin(); it!=robot_motor_current_log_.end(); it++ )
      {
        Json::Value root;
        Json::StreamWriterBuilder builder;
        std::string json_file;
        char jnt_name[10];

        //char arm[10]; 
        sprintf(arm,"%d",it->first);

        root["time"] = it->second.time_us;

        ///////////////////////////////////
        // Motor currents
        // uint32_t payload_len_t = (uint32_t) sizeof(it->second.time_us)*2;
        // uint32_t payload_len_d = (uint32_t) sizeof(it->second.value); 
        // payload_len_ = payload_len_t + payload_len_d;

        
        // memset( payload_, 0, MAX_PAYLOAD_SIZE );
        // snprintf( (char*)payload_, MAX_PAYLOAD_SIZE, "%lld", it->second.time_us );

        // for (size_t idx=0; idx<payload_len_/SIZE_OF_DOUBLE; idx++)
        //   snprintf ( (char*)payload_ + payload_len_t + SIZE_OF_DOUBLE * idx , MAX_PAYLOAD_SIZE, "%f", it->second.value[idx] );

        // topic_name_ = "robot/arm" + std::string(arm) + "/motor_currents";
        // if ( publish( payload_, payload_len_, topic_name_ ) != MOSQ_ERR_SUCCESS )
        //   return false;

        topic_name_ = "robot/arm" + std::string(arm) + "/motor_currents";
        
        for (size_t idx=0; idx<ORL_MAX_AXIS; idx++)
        {
          sprintf(jnt_name,"J%d",idx+1); 
          root[jnt_name] =  it->second.target_vel[idx];
        }
        
        json_file = Json::writeString(builder, root);
        payload_len_ = json_file.length() + 1;

        if (payload_len_ < MAX_PAYLOAD_SIZE)
        {
          strcpy(payload_, json_file.c_str());
          
          if ( publish(payload_, payload_len_, topic_name_) != MOSQ_ERR_SUCCESS )
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
      
      std::map<std::string,c5gopen::RobotJointState> last_messages = drapebot_msg_decoder_->getLastReceivedMessage( );

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

        c5gopen_driver->setRobotJointAbsoluteTargetPosition(arm, it->second); 
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