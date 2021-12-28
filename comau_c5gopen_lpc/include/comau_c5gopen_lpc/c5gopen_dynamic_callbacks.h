
#ifndef __COMAU_C5GOPEN_LPC__C5GOPEN_DYNAMIC_CALLBACKS_H__
#define __COMAU_C5GOPEN_LPC__C5GOPEN_DYNAMIC_CALLBACKS_H__

#include <eORL.h>
#include <comau_c5gopen_lpc/c5gopen_driver.h>

namespace c5gopen
{

  class CallbackBase
  {
  public:
    // input: pointer to a unique C callback. 
    CallbackBase(ORLOPEN_callback pCCallback);

    void Free();

    ORLOPEN_callback Reserve(C5GOpenDriver* instance, C5GOpenDriver::CF method);
    
  protected:
    static int StaticInvoke(int int_comau);
      
  private:
    ORLOPEN_callback m_pCCallback;
    C5GOpenDriver* m_pClass;
    C5GOpenDriver::CF m_pMethod;
  };

  template <int context_template> class DynamicCallback : public CallbackBase
  {
  public:
    DynamicCallback(): CallbackBase(&DynamicCallback<context_template>::GeneratedStaticFunction) { }

  private:
    static int GeneratedStaticFunction  (int int_comau)
    {
      return StaticInvoke(int_comau);
    }
  };

  class MemberFunctionCallback
  {
  public:
      
    operator ORLOPEN_callback() const
    {
        return m_cbCallback;
    }

    bool IsValid() const;
    
    MemberFunctionCallback(C5GOpenDriver* instance, C5GOpenDriver::CF method);
    ~MemberFunctionCallback();

  private:
    ORLOPEN_callback m_cbCallback;
    int m_nAllocIndex;

  private:
  //     MemberFunctionCallback( const MemberFunctionCallback& os );
  //     MemberFunctionCallback& operator=( const MemberFunctionCallback& os );
  };

}

#endif