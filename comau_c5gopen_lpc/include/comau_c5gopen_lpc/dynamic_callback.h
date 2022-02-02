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

      on_connect_callback Reserve(MQTTClient* instance, MQTTClient::on_connect_callback method);
  protected:
      static void StaticInvoke(int context,  void *obj, int reason_code);
      
  private:
      on_connect_callback m_pCCallback;
      MQTTClient* m_pClass;
      MQTTClient::on_connect_callback m_pMethod;
  };


  class OnMessageCallbackBase
  {
  public:
      // input: pointer to a unique C callback. 
      OnMessageCallbackBase(on_message_callback pCCallback);
      void Free();
      on_message_callback Reserve(MQTTClient* instance, MQTTClient::on_message_callback method);

  protected:
      static void StaticInvoke(int context,  void *obj, const struct mosquitto_message *msg);
      
  private:
      on_message_callback m_pCCallback;
      MQTTClient* m_pClass;
      MQTTClient::on_message_callback m_pMethod;
  };

  class OnSubscribeCallbackBase
  {
  public:
      // input: pointer to a unique C callback. 
      OnSubscribeCallbackBase(on_subscribe_callback pCCallback);
      void Free();
      on_subscribe_callback Reserve(MQTTClient* instance, MQTTClient::on_subscribe_callback method);

  protected:
      static void StaticInvoke(int context,  void *obj, uint16_t mid, int qos_count, const uint8_t *granted_qos);
      
  private:
      on_subscribe_callback m_pCCallback;
      MQTTClient* m_pClass;
      MQTTClient::on_subscribe_callback m_pMethod;
  };

  class OnPublishCallbackBase
  {
  public:
      // input: pointer to a unique C callback.
      OnPublishCallbackBase(on_publish_callback pCCallback);
      void Free();
      on_publish_callback Reserve(MQTTClient* instance, MQTTClient::on_publish_callback method);

  protected:
      static void StaticInvoke(int context, void *obj, uint16_t mid);
      
  private:
      on_publish_callback m_pCCallback;
      MQTTClient* m_pClass;
      MQTTClient::on_publish_callback m_pMethod;
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
      
      OnConnectMemberFunctionCallback(MQTTClient* instance, MQTTClient::on_connect_callback method);
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
      
      OnMessageMemberFunctionCallback(MQTTClient* instance, MQTTClient::on_message_callback method);
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
      
      OnSubscribeMemberFunctionCallback(MQTTClient* instance, MQTTClient::on_subscribe_callback method);
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
      
      OnPublishMemberFunctionCallback(MQTTClient* instance, MQTTClient::on_publish_callback method);
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
