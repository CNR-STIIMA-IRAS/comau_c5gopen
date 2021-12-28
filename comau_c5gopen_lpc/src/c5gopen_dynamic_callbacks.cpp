
#include <comau_c5gopen_lpc/c5gopen_dynamic_callbacks.h>

namespace c5gopen
{
  CallbackBase* AvailableCallback = new DynamicCallback<0>();

  CallbackBase::CallbackBase(ORLOPEN_callback pCCallback) : m_pClass( NULL ), m_pMethod( NULL ), m_pCCallback( pCCallback )
  {
  }

  void CallbackBase::Free()
  {
    m_pClass = NULL;
  }

  ORLOPEN_callback CallbackBase::Reserve(C5GOpenDriver* instance, C5GOpenDriver::CF method)
  { 
    if( m_pClass )
      return NULL;

    m_pClass = instance;
    m_pMethod = method;
    return m_pCCallback;
  }

  int CallbackBase::StaticInvoke(int int_comau)
  {
    return ((AvailableCallback->m_pClass)->*(AvailableCallback->m_pMethod)) (int_comau);
  }

  bool MemberFunctionCallback::IsValid() const
  {
    return m_cbCallback != NULL;
  }
  
  MemberFunctionCallback::MemberFunctionCallback(C5GOpenDriver* instance, C5GOpenDriver::CF method)
  {
    m_cbCallback = AvailableCallback->Reserve(instance, method);
    if( !IsValid() )
    {
      AvailableCallback->Free();
      throw std::runtime_error("weird error.");
    }
  }

  MemberFunctionCallback::~MemberFunctionCallback()
  {
    if( IsValid() )
    {
      AvailableCallback->Free();
    }
  }

}