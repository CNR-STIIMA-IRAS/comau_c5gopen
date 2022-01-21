#include <comau_c5gopen_lpc/mqtt.h>
#include <comau_c5gopen_lpc/dynamic_callback.h>

OnConnectCallbackBase* AvailableOnConnectCallbackSlots[] = {
    new OnConnectDynamicCallback<0x00>()
};
OnMessageCallbackBase* AvailableOnMessageCallbackSlots[] = {
    new OnMessageDynamicCallback<0x00>()
};
OnSubscribeCallbackBase* AvailableOnSubscribeCallbackSlots[] = {
    new OnSubscribeDynamicCallback<0x00>()
};
OnPublishCallbackBase* AvailableOnPublishCallbackSlots[] = {
    new OnPublishDynamicCallback<0x00>()
};

// ==============
OnConnectCallbackBase::OnConnectCallbackBase(on_connect_callback pCCallback) : 
m_pClass( NULL ),m_pMethod( NULL ),m_pCCallback( pCCallback )
{
}
void OnConnectCallbackBase::Free()
{
    m_pClass = NULL;
}


on_connect_callback OnConnectCallbackBase::Reserve(mqtt_client* instance, mqtt_client::on_connect_callback method)
{ 
    if( m_pClass )
        return NULL;

    m_pClass = instance;
    m_pMethod = method;
    return m_pCCallback;
}

void OnConnectCallbackBase::StaticInvoke(int context, void *obj, int reason_code)
{
    auto p = (AvailableOnConnectCallbackSlots[context]->m_pClass);
    return (p->*(AvailableOnConnectCallbackSlots[context]->m_pMethod)) (obj, reason_code);
}

// ==============
OnMessageCallbackBase::OnMessageCallbackBase(on_message_callback pCCallback) : 
m_pClass( NULL ),m_pMethod( NULL ),m_pCCallback( pCCallback )
{
}
void OnMessageCallbackBase::Free()
{
    m_pClass = NULL;
}


on_message_callback OnMessageCallbackBase::Reserve(mqtt_client* instance, mqtt_client::on_message_callback method)
{ 
    if( m_pClass )
        return NULL;

    m_pClass = instance;
    m_pMethod = method;
    return m_pCCallback;
}

void OnMessageCallbackBase::StaticInvoke(int context,  void *obj, const struct mosquitto_message *msg)
{
    auto p = (AvailableOnMessageCallbackSlots[context]->m_pClass); 
    return (p->*(AvailableOnMessageCallbackSlots[context]->m_pMethod)) (obj, msg);
}

// ==============
OnSubscribeCallbackBase::OnSubscribeCallbackBase(on_subscribe_callback pCCallback) : 
m_pClass( NULL ),m_pMethod( NULL ),m_pCCallback( pCCallback )
{
}
void OnSubscribeCallbackBase::Free()
{
    m_pClass = NULL;
}


on_subscribe_callback OnSubscribeCallbackBase::Reserve(mqtt_client* instance, mqtt_client::on_subscribe_callback method)
{ 
    if( m_pClass )
        return NULL;

    m_pClass = instance;
    m_pMethod = method;
    return m_pCCallback;
}

void OnSubscribeCallbackBase::StaticInvoke(int context,  void *obj, uint16_t mid, int qos_count, const uint8_t *granted_qos)
{
    auto p = (AvailableOnSubscribeCallbackSlots[context]->m_pClass);
    return (p->*(AvailableOnSubscribeCallbackSlots[context]->m_pMethod)) (obj, mid, qos_count, granted_qos);
}

// ==============
OnPublishCallbackBase::OnPublishCallbackBase(on_publish_callback pCCallback) : 
m_pClass( NULL ),m_pMethod( NULL ),m_pCCallback( pCCallback )
{
}
void OnPublishCallbackBase::Free()
{
    m_pClass = NULL;
}

on_publish_callback OnPublishCallbackBase::Reserve(mqtt_client* instance, mqtt_client::on_publish_callback method)
{
    if( m_pClass )
        return NULL;

    m_pClass = instance;
    m_pMethod = method;
    return m_pCCallback;
}

void OnPublishCallbackBase::StaticInvoke(int context,  void *obj, uint16_t mid)
{
    auto p = (AvailableOnPublishCallbackSlots[context]->m_pClass);
    return (p->*(AvailableOnPublishCallbackSlots[context]->m_pMethod)) (obj, mid);
}



// ***********
bool OnConnectMemberFunctionCallback::IsValid() const
{
    return m_cbCallback != NULL;
}

OnConnectMemberFunctionCallback::OnConnectMemberFunctionCallback(mqtt_client* instance, mqtt_client::on_connect_callback method)
{
    int imax = sizeof(AvailableOnConnectCallbackSlots)/sizeof(AvailableOnConnectCallbackSlots[0]);
    for( m_nAllocIndex = 0; m_nAllocIndex < imax; ++m_nAllocIndex )
    {
    m_cbCallback = AvailableOnConnectCallbackSlots[m_nAllocIndex]->Reserve(instance, method);
    if( m_cbCallback != NULL )
        break;
    }
}
OnConnectMemberFunctionCallback::~OnConnectMemberFunctionCallback()
{
    if( IsValid() )
    {
    AvailableOnConnectCallbackSlots[m_nAllocIndex]->Free();
    }
}

// ***********
bool OnMessageMemberFunctionCallback::IsValid() const
{
    return m_cbCallback != NULL;
}

OnMessageMemberFunctionCallback::OnMessageMemberFunctionCallback(mqtt_client* instance, mqtt_client::on_message_callback method)
{
    int imax = sizeof(AvailableOnMessageCallbackSlots)/sizeof(AvailableOnMessageCallbackSlots[0]);
    for( m_nAllocIndex = 0; m_nAllocIndex < imax; ++m_nAllocIndex )
    {
    m_cbCallback = AvailableOnMessageCallbackSlots[m_nAllocIndex]->Reserve(instance, method);
    if( m_cbCallback != NULL )
        break;
    }
}

OnMessageMemberFunctionCallback::~OnMessageMemberFunctionCallback()
{
    if( IsValid() )
    {
    AvailableOnMessageCallbackSlots[m_nAllocIndex]->Free();
    }
}

// ***********
bool OnSubscribeMemberFunctionCallback::IsValid() const
{
    return m_cbCallback != NULL;
}

OnSubscribeMemberFunctionCallback::OnSubscribeMemberFunctionCallback(mqtt_client* instance, mqtt_client::on_subscribe_callback method)
{
    int imax = sizeof(AvailableOnSubscribeCallbackSlots)/sizeof(AvailableOnSubscribeCallbackSlots[0]);
    for( m_nAllocIndex = 0; m_nAllocIndex < imax; ++m_nAllocIndex )
    {
    m_cbCallback = AvailableOnSubscribeCallbackSlots[m_nAllocIndex]->Reserve(instance, method);
    if( m_cbCallback != NULL )
        break;
    }
}
OnSubscribeMemberFunctionCallback::~OnSubscribeMemberFunctionCallback()
{
    if( IsValid() )
    {
    AvailableOnSubscribeCallbackSlots[m_nAllocIndex]->Free();
    }
}

// ******************************************************************
bool OnPublishMemberFunctionCallback::IsValid() const
{
    return m_cbCallback != NULL;
}

OnPublishMemberFunctionCallback::OnPublishMemberFunctionCallback(mqtt_client* instance, mqtt_client::on_publish_callback method)
{
    int imax = sizeof(AvailableOnPublishCallbackSlots)/sizeof(AvailableOnPublishCallbackSlots[0]);
    for( m_nAllocIndex = 0; m_nAllocIndex < imax; ++m_nAllocIndex )
    {
    m_cbCallback = AvailableOnPublishCallbackSlots[m_nAllocIndex]->Reserve(instance, method);
    if( m_cbCallback != NULL )
        break;
    }
}
OnPublishMemberFunctionCallback::~OnPublishMemberFunctionCallback()
{
    if( IsValid() )
    {
        AvailableOnPublishCallbackSlots[m_nAllocIndex]->Free();
    }
}

