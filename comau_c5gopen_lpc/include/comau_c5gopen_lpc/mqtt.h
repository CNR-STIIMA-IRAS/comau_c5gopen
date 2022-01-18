#ifndef SIMPLECLIENT_MQTT_H
#define SIMPLECLIENT_MQTT_H

#include <mosquitto.h>
#include <cstring>
#include <cstdio>

#define MAX_PAYLOAD 50
#define DEFAULT_KEEP_ALIVE 60

class mqtt_client 
{
public:
    mqtt_client (const char *id, const char *host, int port);
    ~mqtt_client();

    int loop();
    int stop() {return stop_raised=1;}

    int reconnect(unsigned int reconnect_delay, unsigned int reconnect_delay_max, bool reconnect_exponential_backoff);
    int subscribe(uint16_t *mid, const char *sub, int qos=0);
    void publish();

    typedef void (mqtt_client::*on_connect_callback)  (void *obj, int reason_code);
    typedef void (mqtt_client::*on_message_callback)  (void *obj, const struct mosquitto_message *msg);
    typedef void (mqtt_client::*on_subscribe_callback)(void *obj, uint16_t mid, int qos_count, const uint8_t *granted_qos);
    typedef void (mqtt_client::*on_publish_callback)  (void *obj, uint16_t mid);

    void on_connect  (void *obj, int reason_code);
    void on_message  (void *obj, const struct mosquitto_message *msg);
    void on_subscribe(void *obj, uint16_t mid, int qos_count, const uint8_t *granted_qos);
    void on_publish  (void *obj, uint16_t mid);

private: 
    mosquitto *mosq;
    uint8_t obj[1024];
    int stop_raised = 0;
};

#endif //SIMPLECLIENT_MQTT_H
