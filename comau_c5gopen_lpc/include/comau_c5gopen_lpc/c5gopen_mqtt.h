
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

namespace c5gopen
{
  class C5GOpenMsgDecoder: public cnr::mqtt::MsgDecoder
  {
  public:
    C5GOpenMsgDecoder() {};
    
    // The method should be reimplemented on the base of the application
    void on_message(const struct mosquitto_message *msg) override;
    bool isNewMessageAvailable();
    std::map<std::string,c5gopen::RobotJointState> getLastReceivedMessage( );

    std::mutex mqtt_mtx_;

  private:
    std::map<std::string,c5gopen::RobotJointState> last_received_msg_;
    bool new_message_available_;
  };

  class C5GOpenMQTT 
  {
  private:
    uint8_t payload_[MAX_PAYLOAD_SIZE]; 
    uint32_t payload_len_ = 0;
    std::string topic_name_;

    size_t loop_timeout_;

    typedef std::chrono::high_resolution_clock Clock;
    typedef std::chrono::microseconds microsec;

    std::shared_ptr<c5gopen::C5GOpenDriver> c5gopen_driver_;
    std::shared_ptr<cnr_logger::TraceLogger> logger_;

    cnr::mqtt::MQTTClient* mqtt_client_; 
    c5gopen::C5GOpenMsgDecoder* c5gopen_msg_decoder_;

    std::thread mqtt_thread_;
    thread_status mqtt_thread_status_ = thread_status::BEFORE_RUN; 

    void MQTTThread( );

  public:
    C5GOpenMQTT ( const char *id, const char *host, int port, 
                  const size_t& loop_timeout,
                  const std::shared_ptr<c5gopen::C5GOpenDriver>& c5gopen_driver,
                  const std::shared_ptr<cnr_logger::TraceLogger>& logger); 
    ~C5GOpenMQTT(); 

    bool publishData( );
    bool subscribeTopic( const std::string& sub_topic_name );
    bool updateRobotTargetTrajectory( );
    thread_status getMQTTThreadsStatus();
  }; 

} // end namespace

#endif //SIMPLECLIENT_MQTT_H
