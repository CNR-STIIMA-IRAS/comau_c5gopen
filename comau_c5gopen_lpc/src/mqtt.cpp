#include <iostream>
#include <thread>
#include <ctime>
#include  <stdexcept>
#include <chrono>
#include <vector>

#include <comau_c5gopen_lpc/mqtt.h>
#include <comau_c5gopen_lpc/dynamic_callback.h>
#define PUBLISH_TOPIC "EXAMPLE_TOPIC"

#ifdef DEBUG
#include <iostream>
#endif

namespace cnr
{
    size_t N_SAMPLES = 1000;
    size_t DELTA_US = 1e5;
    std::vector<double> log_mean;
    std::vector<double> log_max;
    char errbuffer[1024] = {0};
    
    mqtt_client::mqtt_client(const char *id, const char *host, int port) 
    {
      int rc;

      /* Required before calling other mosquitto functions */
      mosquitto_lib_init();

      /* Create a new client instance.
      * id = NULL -> ask the broker to generate a client id for us
      * clean session = true -> the broker should remove old sessions when we connect
      * obj = NULL -> we aren't passing any of our private data for callbacks
      */
      mosq = mosquitto_new(id, obj);
      if(mosq == NULL)
      {
        fprintf(stderr, "Error: Out of memory.\n");
        throw std::runtime_error("Error: Out of memory.");
      }

      log_mean.reserve(N_SAMPLES);
      log_max.reserve(N_SAMPLES);


      /* Configure callbacks. This should be done before connecting ideally. */
      mosquitto_connect_callback_set(mosq, OnConnectMemberFunctionCallback(this, &mqtt_client::on_connect));
      mosquitto_subscribe_callback_set(mosq, OnSubscribeMemberFunctionCallback(this, &mqtt_client::on_subscribe));
      mosquitto_message_callback_set(mosq, OnMessageMemberFunctionCallback(this, &mqtt_client::on_message));
      mosquitto_publish_callback_set(mosq, OnPublishMemberFunctionCallback(this, &mqtt_client::on_publish));

        /* Connect to test.mosquitto.org on port 1883, with a keepalive of 60 seconds.
      * This call makes the socket connection only, it does not complete the MQTT
      * CONNECT/CONNACK flow, you should use mosquitto_loop_start() or
      * mosquitto_loop_forever() for processing net traffic. */
      rc = mosquitto_connect(mosq, host, 1883, 60, true);
      if(rc != MOSQ_ERR_SUCCESS)
      {
        mosquitto_destroy(mosq);
        fprintf(stderr, "Error: %s\n", strerror_r(rc,errbuffer,1024));
        throw std::runtime_error("Error");
      }
    }


    int mqtt_client::loop()
    {
      /* Run the network loop in a blocking call. The only thing we do in this
      * example is to print incoming messages, so a blocking call here is fine.
      *
      * This call will continue forever, carrying automatic reconnections if
      * necessary, until the user calls mosquitto_disconnect().
      */
      // return mosquitto_loop_forever(mosq, -1, 1);

      int rc = 0;
      while(!stop_raised)
      {
        /* Run the network loop in a background thread, this call returns quickly. */
        rc = mosquitto_loop(mosq,1);
        if(rc != MOSQ_ERR_SUCCESS)
        {
          mosquitto_destroy(mosq);
          fprintf(stderr, "Error: %s\n", strerror_r(rc, errbuffer,1024));
          return 1;
        }

        /* At this point the client is connected to the network socket, but may not
        * have completed CONNECT/CONNACK.
        * It is fairly safe to start queuing messages at this point, but if you
        * want to be really sure you should wait until after a successful call to
        * the connect callback.
        * In this case we know it is 1 second before we start publishing.
        */

        publish();
        //std::this_thread::sleep_for(std::chrono::microseconds(1));
      }

      return rc;
    }
    int mqtt_client::reconnect(unsigned int reconnect_delay, unsigned int reconnect_delay_max, bool reconnect_exponential_backoff)
    {
      return -1; //mosquitto_reconnect_delay_set(mosq,reconnect_delay, reconnect_delay_max, reconnect_exponential_backoff);
    }
        
    int mqtt_client::subscribe(uint16_t *mid, const char *sub, int qos)
    {
      /* Making subscriptions in the on_connect() callback means that if the
      * connection drops and is automatically resumed by the client, then the
      * subscriptions will be recreated when the client reconnects. */
      int rc = mosquitto_subscribe(mosq, mid, sub, qos);
      if(rc != MOSQ_ERR_SUCCESS)
      {
        fprintf(stderr, "Error subscribing: %s\n", strerror_r(rc, errbuffer,1024));
        /* We might as well disconnect if we were unable to subscribe */
        mosquitto_disconnect(mosq);
      }
      return rc;
    }

    /* This function pretends to read some data from a sensor and publish it.*/
    void mqtt_client::publish()
    {
      static uint8_t payload[20];
      int temp = 100;
      int rc;

      /* Print it to a string for easy human reading - payload format is highly
      * application dependent. */
      snprintf((char*)payload, sizeof(payload), "%d", temp);

      /* Publish the message
      * mosq - our client instance
      * *mid = NULL - we don't want to know what the message id for this message is
      * topic = "example/temperature" - the topic on which this message will be published
      * payloadlen = strlen(payload) - the length of our payload in bytes
      * payload - the actual payload
      * qos = 2 - publish with QoS 2 for this example
      * retain = false - do not use the retained message feature for this message
      */
      rc = mosquitto_publish(mosq, NULL, "example/temperature", strlen((const char*)payload), payload, 0, false);
      if(rc != MOSQ_ERR_SUCCESS){
          fprintf(stderr, "Error publishing: %s\n", strerror_r(rc, errbuffer,1024));
      }
    }


    mqtt_client::~mqtt_client()
    {
      /* Run the network loop in a blocking call. The only thing we do in this
      * example is to print incoming messages, so a blocking call here is fine.
      *
      * This call will continue forever, carrying automatic reconnections if
      * necessary, until the user calls mosquitto_disconnect().
      */
      for(size_t i=0;i<log_mean.size();i++)
      {
          std::cout << "[" << std::fixed << log_mean.at(i) << "us / " << log_max.at(i) << "us ]\n";
      }
      std::cout << std::endl;
      //mosquitto_loop_forever(mosq, -1, 1);
      mosquitto_lib_cleanup();
    }

    void mqtt_client::on_connect(void *obj, int reason_code)
    {
      int rc;
      /* Print out the connection result. mosquitto_connack_string() produces an
      * appropriate string for MQTT v3.x clients, the equivalent for MQTT v5.0
      * clients is mosquitto_reason_string().
      */
      printf("on_connect:");// %s\n", mosquitto_connack_string(reason_code));
      if(reason_code != 0)
      {
        /* If the connection fails for any reason, we don't want to keep on
        * retrying in this example, so disconnect. Without this, the client
        * will attempt to reconnect. */
        mosquitto_disconnect(mosq);
      }
    }

    void mqtt_client::on_subscribe(void *obj, uint16_t mid, int qos_count, const uint8_t *granted_qos)
    {
      int i;
      bool have_subscription = false;

      /* In this example we only subscribe to a single topic at once, but a
          * SUBSCRIBE can contain many topics at once, so this is one way to check
          * them all. */
      for(i=0; i<qos_count; i++)
      {
        printf("on_subscribe: %d:granted qos = %d\n", i, granted_qos[i]);
        if(granted_qos[i] <= 2)
        {
          have_subscription = true;
        }
      }
      if(have_subscription == false)
      {
        /* The broker rejected all of our subscriptions, we know we only sent
            * the one SUBSCRIBE, so there is no point remaining connected. */
        fprintf(stderr, "Error: All subscriptions rejected.\n");
        mosquitto_disconnect(mosq);
      }
    }

    void mqtt_client::on_publish(void *obj, uint16_t mid)
    {
      static long long delta = 0;
      static long long max = 0;
      static int cycle = 0;
      static int window = 0;
      typedef std::chrono::high_resolution_clock Clock;
      typedef std::chrono::microseconds microsec;

      static Clock::time_point old = Clock::now();
      Clock::time_point now = Clock::now();
      microsec us = std::chrono::duration_cast<microsec>(now - old);

      cycle++;
      delta += us.count();
      if(us.count()>max)
      {
        max = us.count();
      }

      if(delta > DELTA_US)
      {
        log_mean.push_back(delta/cycle);
        log_max.push_back(max);
        delta = 0;
        max = 0;
        cycle = 0;
      }
      old = now;
    }

    void mqtt_client::on_message(void *obj, const struct mosquitto_message *msg)
    {
      /* This blindly prints the payload, but the payload can be anything so take care. */
      printf("%s %d %s\n", msg->topic, msg->qos, (char *)msg->payload);
    }

} // end namespace cnr
