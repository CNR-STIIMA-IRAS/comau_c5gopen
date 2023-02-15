
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

#ifndef SIMPLECLIENT_MQTT_H
#define SIMPLECLIENT_MQTT_H

#include <cstring>
#include <cstdio>
#include <mutex>
#include <mosquitto.h>

#include <jsoncpp/json/json.h>

#include <boost/circular_buffer.hpp>
#include <cnr_logger/cnr_logger.h>

#include <comau_c5gopen_lpc/c5gopen_utilities.h>

#define MAX_PAYLOAD_SIZE 1024
#define DEFAULT_KEEP_ALIVE 60
#define LOG_SAMPLES 1000
#define DELTA_US 1e5

namespace cnr
{
  // struct MQTTPayload
  // { 
  //   char payload[MAX_PAYLOAD_SIZE] = {0};  
  // };


  class MQTTClient 
  {
  protected:  
    std::shared_ptr<cnr_logger::TraceLogger> logger_;

  private: 
    struct mosquitto *mosq_;
    uint8_t obj_[1024];
    int stop_raised_ = 0; 
    char errbuffer_[1024] = {0};

  public:
    MQTTClient (const char *id, const char *host, int port, const std::shared_ptr<cnr_logger::TraceLogger>& logger);
    ~MQTTClient();

    int loop(const int& timeout=1000);
    int stop() {return stop_raised_ = 1;}

    int reconnect(unsigned int reconnect_delay, unsigned int reconnect_delay_max, bool reconnect_exponential_backoff);
    int subscribe(uint16_t *mid, const char *sub, int qos=0);
    int publish(const uint8_t* payload, const uint32_t& payload_len, const std::string& topic_name);
    //std::map<std::string,std::pair<int,struct cnr::MQTTPayload>> getLastReceivedMessage( );
    std::map<std::string,c5gopen::RobotJointState> getLastReceivedMessage( );

    typedef void (MQTTClient::*on_connect_callback)  (void *obj, int reason_code);
    typedef void (MQTTClient::*on_message_callback)  (void *obj, const struct mosquitto_message *msg);
    typedef void (MQTTClient::*on_subscribe_callback)(void *obj, uint16_t mid, int qos_count, const uint8_t *granted_qos);
    typedef void (MQTTClient::*on_publish_callback)  (void *obj, uint16_t mid);

    void on_connect  (void *obj, int reason_code);
    void on_message  (void *obj, const struct mosquitto_message *msg);
    void on_subscribe(void *obj, uint16_t mid, int qos_count, const uint8_t *granted_qos);
    void on_publish  (void *obj, uint16_t mid);

  }; 
} // end namespace

#endif //SIMPLECLIENT_MQTT_H
