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

#include <iostream>
#include <thread>
#include <ctime>
#include <stdexcept>
#include <chrono>
#include <vector>

#include <comau_c5gopen_lpc/mqtt.h>
#include <comau_c5gopen_lpc/dynamic_callback.h>

namespace cnr
{  
  namespace mqtt
  {
    double delta_pub_;
    int64_t max_pub_;
    boost::circular_buffer<int64_t> delta_time_(LOG_SAMPLES);

    cnr::mqtt::MsgDecoder* g_msg_decoder;

    int init_library(cnr::mqtt::MsgDecoder* msg_decoder)
    {
      if (msg_decoder != NULL)
      {
        g_msg_decoder = msg_decoder;
        return 0;
      }
      else
        return -1;  
    }

    MQTTClient::MQTTClient( const char *id, const char *host, int port, 
                            cnr::mqtt::MsgDecoder* msg_decoder,
                            const std::shared_ptr<cnr_logger::TraceLogger>& logger ):
                            logger_(logger)   
    {
      //delta_time_.set_capacity(LOG_SAMPLES);

      /* Required before calling other mosquitto functions */
      mosquitto_lib_init();

      /* Create a new client instance.
      * id = NULL -> ask the broker to generate a client id for us
      * obj = NULL -> we aren't passing any of our private data for callbacks
      */
      mosq_ = mosquitto_new(id, obj_);
      if(mosq_ == NULL)
      {
        CNR_ERROR(logger_, "Error: Out of memory.");
        throw std::runtime_error("Error: Out of memory.");
      }

      /* Configure callbacks. This should be done before connecting ideally. */
      mosquitto_connect_callback_set(mosq_, OnConnectMemberFunctionCallback(this, &MQTTClient::on_connect));
      mosquitto_subscribe_callback_set(mosq_, OnSubscribeMemberFunctionCallback(this, &MQTTClient::on_subscribe));
      mosquitto_message_callback_set(mosq_, OnMessageMemberFunctionCallback(this, &MQTTClient::on_message));
      mosquitto_publish_callback_set(mosq_, OnPublishMemberFunctionCallback(this, &MQTTClient::on_publish));

        /* Connect to test.mosquitto.org on port 1883, with a keepalive of 60 seconds.
      * This call makes the socket connection only, it does not complete the MQTT
      * CONNECT/CONNACK flow, you should use mosquitto_loop_start() or
      * mosquitto_loop_forever() for processing net traffic. */

      int rc = mosquitto_connect(mosq_, host, port, 60, true);
      if( rc != MOSQ_ERR_SUCCESS )
      {
        mosquitto_destroy(mosq_);
        CNR_ERROR(logger_, "Error: " << strerror_r(rc,errbuffer_,1024) );
        throw std::runtime_error("Error");
      }
    }


    int MQTTClient::loop(const int& timeout)
    {
      /* Run the network loop in a blocking call. The only thing we do in this
      * example is to print incoming messages, so a blocking call here is fine.
      *
      * This call will continue forever, carrying automatic reconnections if
      * necessary, until the user calls mosquitto_disconnect().
      */
      // return mosquitto_loop_forever(mosq, -1, 1);
      
      int rc = 0;
      /* Run the network loop in a background thread, this call returns quickly. */
      rc = mosquitto_loop(mosq_,timeout);
      if(rc != MOSQ_ERR_SUCCESS)
        CNR_ERROR(logger_, "Error in mosquitto_loop: " << strerror_r(rc, errbuffer_,1024) );

      /* At this point the client is connected to the network socket, but may not
      * have completed CONNECT/CONNACK.
      * It is fairly safe to start queuing messages at this point, but if you
      * want to be really sure you should wait until after a successful call to
      * the connect callback.
      * In this case we know it is 1 second before we start publishing.
      */

      return rc;
    }

    int MQTTClient::reconnect(unsigned int reconnect_delay, unsigned int reconnect_delay_max, bool reconnect_exponential_backoff)
    {
      return -1; //mosquitto_reconnect_delay_set(mosq,reconnect_delay, reconnect_delay_max, reconnect_exponential_backoff);
    }
        
    int MQTTClient::subscribe(uint16_t *mid, const char *sub, int qos)
    {
      /* Making subscriptions in the on_connect() callback means that if the
      * connection drops and is automatically resumed by the client, then the
      * subscriptions will be recreated when the client reconnects. */
      int rc = mosquitto_subscribe(mosq_, mid, sub, qos);
      if(rc != MOSQ_ERR_SUCCESS)
      {
        CNR_WARN(logger_, "Error on topic subscription: " << strerror_r(rc, errbuffer_,1024) );
        /* We might as well disconnect if we were unable to subscribe */
        mosquitto_disconnect(mosq_);
      }
      return rc;
    }

    /* This function pretends to read some data from a sensor and publish it.*/
    int MQTTClient::publish( const uint8_t* payload, const uint32_t& payload_len, const std::string& topic_name )
    {
      /* Publish the message
      * mosq - our client instance 
      * *mid = NULL - we don't want to know what the message id for this message is
      * topic = "example/temperature" - the topic on which this message will be published
      * payloadlen = strlen(payload) - the length of our payload in bytes
      * payload - the actual payload
      * qos = 2 - publish with QoS 2 for this example
      * retain = false - do not use the retained message feature for this message
      */
      int rc = mosquitto_publish(mosq_, NULL, topic_name.c_str(), payload_len, payload, 0, false);
      if( rc != MOSQ_ERR_SUCCESS )
      {
        CNR_ERROR(logger_, "Error publishing: " << strerror_r(rc, errbuffer_,1024) );
      }
      return rc;
    }

    MQTTClient::~MQTTClient()
    {
      /* Run the network loop in a blocking call. The only thing we do in this
      * example is to print incoming messages, so a blocking call here is fine.
      *
      * This call will continue forever, carrying automatic reconnections if
      * necessary, until the user calls mosquitto_disconnect().
      */

      CNR_INFO(logger_, "Time to publish message [ average / max ] us " );    
      CNR_INFO(logger_, "[" << std::fixed << delta_pub_ << " / " << max_pub_ << " ] us" );
      
      mosquitto_lib_cleanup();
    }

    void MQTTClient::on_connect(void *obj, int reason_code)
    {
      /* Print out the connection result. mosquitto_connack_string() produces an
      * appropriate string for MQTT v3.x clients, the equivalent for MQTT v5.0
      * clients is mosquitto_reason_string().
      */
      if(reason_code != 0)
      {
        /* If the connection fails for any reason, we don't want to keep on
        * retrying in this example, so disconnect. Without this, the client
        * will attempt to reconnect. */
        mosquitto_disconnect(mosq_);
      }
    }

    void MQTTClient::on_subscribe(void *obj, uint16_t mid, int qos_count, const uint8_t *granted_qos)
    {
      bool have_subscription = false;
      /*  In this example we only subscribe to a single topic at once, but a
          SUBSCRIBE can contain many topics at once, so this is one way to check them all. */
      for( int i=0; i<qos_count; i++)
      {
        if(granted_qos[i] <= 2)
          have_subscription = true;
      }

      if( have_subscription == false )
      {
        /* The broker rejected all of our subscriptions, we know we only sent
          the one SUBSCRIBE, so there is no point remaining connected. */
        mosquitto_disconnect(mosq_);
      }
    }

    void MQTTClient::on_publish(void *obj, uint16_t mid)
    { 
      static int cycle = 0;
      static bool buffer_full = false;
      typedef std::chrono::high_resolution_clock Clock;
      typedef std::chrono::microseconds microsec;

      static Clock::time_point old = Clock::now();
      Clock::time_point now = Clock::now();
      microsec us = std::chrono::duration_cast<microsec>(now - old);

      if( us.count() > max_pub_ )  
        max_pub_ = us.count();
    
      delta_time_.push_back(us.count());
  
      int64_t cum_delta = 0;
      for (boost::circular_buffer<int64_t>::const_iterator it = delta_time_.begin(); it != delta_time_.end(); it++)
        cum_delta += *it;

      cycle++;
      
      if ( cycle < LOG_SAMPLES && !buffer_full )
        delta_pub_ = (double) cum_delta / cycle;  
      else
      {
        delta_pub_ = (double) cum_delta / LOG_SAMPLES;
        buffer_full = true;
      }

      old = now;
    }

    void MQTTClient::on_message(void *obj, const struct mosquitto_message *msg)
    {
      g_msg_decoder->on_message( msg );
    }
  } // end namespace mqtt
} // end namespace cnr
