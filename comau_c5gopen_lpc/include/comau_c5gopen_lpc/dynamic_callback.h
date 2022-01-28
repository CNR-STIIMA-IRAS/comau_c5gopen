
#ifndef DYNAMIC_CALLBACK_MQTT_H
#define DYNAMIC_CALLBACK_MQTT_H

#include <comau_c5gopen_lpc/mqtt.h>

namespace cnr
{
  typedef void (*on_connect_callback)(void *obj, int reason_code);
  typedef void (*on_message_callback)( void *obj, const struct mosquitto_message *msg);
  typedef void (*on_subscribe_callback)( void *obj, uint16_t mid, int qos_count, const uint8_t *granted_qos);
  typedef void (*on_publish_callback)( void *obj, uint16_t mid);

  class OnConnectCallbackBase
  {
  public:
      // input: pointer to a unique C callback. 
      OnConnectCallbackBase(on_connect_callback pCCallback);
      void Free();

      on_connect_callback Reserve(mqtt_client* instance, mqtt_client::on_connect_callback method);
  protected:
      static void StaticInvoke(int context,  void *obj, int reason_code);
      
  private:
      on_connect_callback m_pCCallback;
      mqtt_client* m_pClass;
      mqtt_client::on_connect_callback m_pMethod;
  };


  class OnMessageCallbackBase
  {
  public:
      // input: pointer to a unique C callback. 
      OnMessageCallbackBase(on_message_callback pCCallback);
      void Free();
      on_message_callback Reserve(mqtt_client* instance, mqtt_client::on_message_callback method);

  protected:
      static void StaticInvoke(int context,  void *obj, const struct mosquitto_message *msg);
      
  private:
      on_message_callback m_pCCallback;
      mqtt_client* m_pClass;
      mqtt_client::on_message_callback m_pMethod;
  };

  class OnSubscribeCallbackBase
  {
  public:
      // input: pointer to a unique C callback. 
      OnSubscribeCallbackBase(on_subscribe_callback pCCallback);
      void Free();
      on_subscribe_callback Reserve(mqtt_client* instance, mqtt_client::on_subscribe_callback method);

  protected:
      static void StaticInvoke(int context,  void *obj, uint16_t mid, int qos_count, const uint8_t *granted_qos);
      
  private:
      on_subscribe_callback m_pCCallback;
      mqtt_client* m_pClass;
      mqtt_client::on_subscribe_callback m_pMethod;
  };

  class OnPublishCallbackBase
  {
  public:
      // input: pointer to a unique C callback.
      OnPublishCallbackBase(on_publish_callback pCCallback);
      void Free();
      on_publish_callback Reserve(mqtt_client* instance, mqtt_client::on_publish_callback method);

  protected:
      static void StaticInvoke(int context, void *obj, uint16_t mid);
      
  private:
      on_publish_callback m_pCCallback;
      mqtt_client* m_pClass;
      mqtt_client::on_publish_callback m_pMethod;
  };

  template <int context> class OnConnectDynamicCallback : public OnConnectCallbackBase
  {
  public:
      OnConnectDynamicCallback(): OnConnectCallbackBase(&OnConnectDynamicCallback<context>::GeneratedStaticFunction) { }

  private:
      static void GeneratedStaticFunction( void *obj, int reason_code)
      {
          return StaticInvoke(context, obj,reason_code);
      }
  };

  template <int context> class OnMessageDynamicCallback : public OnMessageCallbackBase
  {
  public:
      OnMessageDynamicCallback(): OnMessageCallbackBase(&OnMessageDynamicCallback<context>::GeneratedStaticFunction) { }

  private:
      static void GeneratedStaticFunction( void *obj, const struct mosquitto_message *msg)
      {
          return StaticInvoke(context , obj,msg);
      }
  };

  template <int context> class OnSubscribeDynamicCallback : public OnSubscribeCallbackBase
  {
  public:
      OnSubscribeDynamicCallback(): OnSubscribeCallbackBase(&OnSubscribeDynamicCallback<context>::GeneratedStaticFunction) { }

  private:
      static void GeneratedStaticFunction( void *obj, uint16_t mid, int qos_count, const uint8_t *granted_qos)
      {
          return StaticInvoke(context, obj, mid, qos_count, granted_qos);
      }
  };


  template <int context> class OnPublishDynamicCallback : public OnPublishCallbackBase
  {
  public:
      OnPublishDynamicCallback(): OnPublishCallbackBase(&OnPublishDynamicCallback<context>::GeneratedStaticFunction) { }

  private:
      static void GeneratedStaticFunction(void *obj, uint16_t mid)
      {
          return StaticInvoke(context, obj, mid);
      }
  };

  class OnConnectMemberFunctionCallback
  {
  public:
      
      operator on_connect_callback() const { return m_cbCallback; }

      bool IsValid() const;
      
      OnConnectMemberFunctionCallback(mqtt_client* instance, mqtt_client::on_connect_callback method);
      ~OnConnectMemberFunctionCallback();

  private:
      on_connect_callback m_cbCallback;
      int m_nAllocIndex;

  private:
  //     MemberFunctionCallback( const MemberFunctionCallback& os );
  //     MemberFunctionCallback& operator=( const MemberFunctionCallback& os );
  };

  class OnMessageMemberFunctionCallback
  {
  public:
      
      operator on_message_callback() const {return m_cbCallback;}

      bool IsValid() const;
      
      OnMessageMemberFunctionCallback(mqtt_client* instance, mqtt_client::on_message_callback method);
      ~OnMessageMemberFunctionCallback();

  private:
      on_message_callback m_cbCallback;
      int m_nAllocIndex;

  private:
  //     MemberFunctionCallback( const MemberFunctionCallback& os );
  //     MemberFunctionCallback& operator=( const MemberFunctionCallback& os );
  };

  class OnSubscribeMemberFunctionCallback
  {
  public:
      
      operator on_subscribe_callback() const {return m_cbCallback;}

      bool IsValid() const;
      
      OnSubscribeMemberFunctionCallback(mqtt_client* instance, mqtt_client::on_subscribe_callback method);
      ~OnSubscribeMemberFunctionCallback();

  private:
      on_subscribe_callback m_cbCallback;
      int m_nAllocIndex;

  private:
  //     MemberFunctionCallback( const MemberFunctionCallback& os );
  //     MemberFunctionCallback& operator=( const MemberFunctionCallback& os );
  };

  class OnPublishMemberFunctionCallback
  {
  public:
      
      operator on_publish_callback() const {return m_cbCallback;}

      bool IsValid() const;
      
      OnPublishMemberFunctionCallback(mqtt_client* instance, mqtt_client::on_publish_callback method);
      ~OnPublishMemberFunctionCallback();

  private:
      on_publish_callback m_cbCallback;
      int m_nAllocIndex;

  private:
  //     MemberFunctionCallback( const MemberFunctionCallback& os );
  //     MemberFunctionCallback& operator=( const MemberFunctionCallback& os );
  };

} // end namespace cnr

#endif
