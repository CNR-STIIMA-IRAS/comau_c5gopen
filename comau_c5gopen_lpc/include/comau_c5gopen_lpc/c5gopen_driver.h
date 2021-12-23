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

#ifndef __C5GOPEN_LPC_NODE__
#define __C5GOPEN_LPC_NODE__

#include <string>
#include <vector>

#include <sched.h>
#include <stdlib.h>

#include <thread>
#include <boost/circular_buffer.hpp>

#include <eORL.h>

#include <cnr_logger/cnr_logger.h>
#include <cnr_logger/cnr_logger_macros.h>

#include <comau_c5gopen_lpc/realtime_buffer_utils.h>

namespace c5gopen
{
  class CallbackBase;
  template<int context> class DynamicCallback;
  typedef int (*CF)(int);

  extern CallbackBase* AvailableCallbackSlots[1];

  class C5GOpenDriver 
  {
  public:
    C5GOpenDriver(const std::string& ip_ctrl, const std::string& sys_id,
                  const int& c5gopen_period, std::shared_ptr<cnr_logger::TraceLogger>& logger ); 

    ~C5GOpenDriver();

    bool init();
    bool run();
    bool getThreadsStatus();
    int c5gopen_callback( int input );

    typedef int (C5GOpenDriver::*CF)(int);
    
  private:

    bool threads_status_;
    bool c5gopen_active_;

    int c5gopen_period_; // in microseconds

    std::string ip_ctrl_;
    std::string sys_id_;

    std::thread c5gopen_thread_; 
    std::thread com_thread_; 
    std::thread loop_console_thread_;
    std::shared_ptr<cnr_logger::TraceLogger> logger_;

    void c5gopen_thread( );
    void com_thread( );
    void loop_console_thread( );

  };


  class CallbackBase
  {
  public:
    // input: pointer to a unique C callback. 
    CallbackBase(CF pCCallback) : 
    m_pClass( NULL ),m_pMethod( NULL ),m_pCCallback( pCCallback )
    {
    }
    void Free()
    {
      m_pClass = NULL;
    }

    CF Reserve(C5GOpenDriver* instance, C5GOpenDriver::CF method)
    { 
      if( m_pClass )
          return NULL;

      m_pClass = instance;
      m_pMethod = method;
      return m_pCCallback;
    }
  protected:
    static int StaticInvoke(int context_template, int int_comau)
    {
        return ((AvailableCallbackSlots[context_template]->m_pClass)->*(AvailableCallbackSlots[context_template]->m_pMethod)) (int_comau);
    }
      
  private:
    CF m_pCCallback;
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
        return StaticInvoke(context_template , int_comau);
    }
  };

  class MemberFunctionCallback
  {
  public:
      
    operator CF() const
    {
        return m_cbCallback;
    }

    bool IsValid() const
    {
        return m_cbCallback != NULL;
    }
    
    MemberFunctionCallback(C5GOpenDriver* instance, C5GOpenDriver::CF method)
    {
      int imax = sizeof(AvailableCallbackSlots)/sizeof(AvailableCallbackSlots[0]);
      for( m_nAllocIndex = 0; m_nAllocIndex < imax; ++m_nAllocIndex )
      {
        m_cbCallback = AvailableCallbackSlots[m_nAllocIndex]->Reserve(instance, method);
        if( m_cbCallback != NULL )
          break;
      }
    }
    ~MemberFunctionCallback()
    {
      if( IsValid() )
      {
        AvailableCallbackSlots[m_nAllocIndex]->Free();
      }
    }

  private:
    CF m_cbCallback;
    int m_nAllocIndex;

  private:
  //     MemberFunctionCallback( const MemberFunctionCallback& os );
  //     MemberFunctionCallback& operator=( const MemberFunctionCallback& os );
  };

} // end of namespace c5gopen






/////////////////////////////// OLD ////////////////////////////


// #define EXTRN  extern

// #define false       0
// #define true        1
// #define LAST_MESS   0

// #define MAX_NUM_ARMS    1          // ATTENTION: define here the number of ARM for ORL library

// #define target_pos_max_buff_len 100
// #define target_pos_max_buff_log_len 300
// #define delta_target_cart_pos_max_len 10

// struct AI_Input 
// {
//   int16_t   ch1_value[ 6 ];
// }__attribute__ ( ( packed ) );


// struct delta_position_t 
// {
//   ORL_cartesian_position ideal;
//   ORL_cartesian_position real;
//   ORL_cartesian_position speed;
// }__attribute__ ( ( packed ) );


// struct absolute_target_position_t
// {
//   ORL_joint_value target_pos;
//   ORL_joint_value target_vel;
// }__attribute__ ( ( packed ) ); 


// struct absolute_real_position_t
// {
//   ORL_joint_value real_pos;
//   ORL_joint_value real_vel;
// }__attribute__ ( ( packed ) ); 


// EXTRN int                         mask_moving_arms;
// EXTRN int                         cycle_active;
// EXTRN bool                        system_initialized;

// EXTRN bool                        first_arm_driveon             [MAX_NUM_ARMS];
// EXTRN bool                        flag_RunningMove              [MAX_NUM_ARMS];
// EXTRN bool                        flag_ExitFromOpen             [MAX_NUM_ARMS];
// EXTRN bool                        trajectory_in_execution       [MAX_NUM_ARMS];
// EXTRN bool                        received_delta_target_cart    [MAX_NUM_ARMS];
// EXTRN bool                        robot_movement_enabled        [MAX_NUM_ARMS];
// EXTRN unsigned int                modality_active               [MAX_NUM_ARMS];
// EXTRN unsigned int                modality_old                  [MAX_NUM_ARMS];

// EXTRN ORL_joint_value             actual_joints_position        [MAX_NUM_ARMS];
// EXTRN ORL_cartesian_position      actual_cartesian_position     [MAX_NUM_ARMS];

// EXTRN realtime_buffer::circ_buffer<delta_position_t> *delta_target_cart_pos[MAX_NUM_ARMS];

// EXTRN realtime_buffer::circ_buffer<absolute_target_position_t> *absolute_target_position[MAX_NUM_ARMS];
// EXTRN realtime_buffer::circ_buffer<absolute_target_position_t> *absolute_target_position_log[MAX_NUM_ARMS];




// void   *loop_console_thread         ( void *shared_thread_data );
// void   *publisher_thread            ( void *shared_thread_data );

// int     user_callback               ( int input );
// int     move_arm                    ( ORL_joint_value* px_target_jnt, int type_move, int idx_cntrl, int idx_arm );
// int     move_cal_sys                ( int idx_cntrl, int idx_arm );
// void    decode_modality             ( const int si_modality, char* string, const bool verbose );
// double  digits_to_double            ( int16_t value, double min_value, double max_value ) ;
// bool    update_robot_status         ( bool verbose=false );
// int     initialize_control_position ( void );
//bool    set_frames                  ( ros::NodeHandle* nh_, ORL_cartesian_position* bFrame, ORL_cartesian_position* tFrame, ORL_cartesian_position* uFrame );
//bool    get_arms_config             ( ros::NodeHandle* nh_, int arms_active );


//****************************************************//

// class C5gopenTrjExec
// {
// private:

//   int                   idxArm_;
//   int                   open_period_;             // ref to eorl.h value are 0,1,2,3,4
//   bool                  robot_movement_enabled_;
//   bool                  trajectory_in_execution_;
//   bool                  received_delta_target_cart_;
  
//   std::string           trj_namespace_;  
  
//   //ros::NodeHandle*      ptr_nh_;
  
//   realtime_buffer::circ_buffer<delta_position_t>* delta_trg_cart_pos_;
//   realtime_buffer::circ_buffer<absolute_target_position_t>* abs_target_pos_;
  
//   //control_msgs::FollowJointTrajectoryResult   res_;
//   //control_msgs::FollowJointTrajectoryFeedback feedback_;
  
//   //ros::Subscriber trj_deb_sub_;
//   //actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> c5gopen_execute_trj_as_;
  
// public:
//   C5gopenTrjExec( const int&            idxArm,
//                   const int&            open_period,
//                   const std::string&    trj_namespace,
//                   //ros::NodeHandle*      ptr_nh,
//                   realtime_buffer::circ_buffer<delta_position_t>* delta_trg_cart_pos,
//                   realtime_buffer::circ_buffer<absolute_target_position_t>* abs_target_pos );
  
//   ~C5gopenTrjExec();
                                        
//   //void execTrjCB    ( const control_msgs::FollowJointTrajectoryGoalConstPtr& trj_goal_ );
//   //void addTrjDefCB  ( const std_msgs::Float64MultiArray::ConstPtr& trj_def_ );
  
//   bool getTrjExecutionFlag( );
//   bool getRobMoveEnabledFlag( );
//   bool getRcvDeltaTargetCartFlag( );
  
// };


#endif
