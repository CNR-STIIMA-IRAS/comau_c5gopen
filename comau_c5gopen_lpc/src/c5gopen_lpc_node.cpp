#include <comau_c5gopen_LPC/c5gopen_lpc_node.h>


// int                         cycle_active            = 0;
// int                         mask_moving_arms        = 0;
// bool                        system_initialized      = false;

// bool                        first_arm_driveon             [MAX_NUM_ARMS] = {false};
// bool                        flag_RunningMove              [MAX_NUM_ARMS] = {false};
// bool                        flag_ExitFromOpen             [MAX_NUM_ARMS] = {false};
// bool                        trajectory_in_execution       [MAX_NUM_ARMS] = {false};
// bool                        received_delta_target_cart    [MAX_NUM_ARMS] = {false};
// bool                        robot_movement_enabled        [MAX_NUM_ARMS] = {false};
// unsigned int                modality_active               [MAX_NUM_ARMS] = {0x0};
// unsigned int                modality_old                  [MAX_NUM_ARMS] = {0x0};

// ORL_joint_value             actual_joints_position        [MAX_NUM_ARMS];
// ORL_cartesian_position      actual_cartesian_position     [MAX_NUM_ARMS];

// itia::butils::circ_buffer<delta_position_t> *delta_target_cart_pos[MAX_NUM_ARMS];

// itia::butils::circ_buffer<absolute_target_position_t> *absolute_target_position[MAX_NUM_ARMS];
// itia::butils::circ_buffer<absolute_target_position_t> *absolute_target_position_log[MAX_NUM_ARMS];

int  main (int argc, char **argv)
{
 
  // int policy = 0;
  // int min_prio_for_policy = 0;
  
  // policy = SCHED_OTHER;  //SCHED_RR, SCHED_FIFO, SCHED_OTHER (POSIX scheduling policies)
  // min_prio_for_policy = sched_get_priority_min(policy);
  
  // struct sched_param main_thread_param;
  
  // memset(&main_thread_param, 0x0, sizeof(sched_param));
  // main_thread_param.sched_priority = min_prio_for_policy;
    
  // if ( pthread_setschedparam( pthread_self(), policy, &main_thread_param ) != 0 )
  //   printf( " [ %s%s:%d%s ]\t %s ERROR in pthread_setschedparam() of main_thread_id%s \n", GREEN, __FUNCFILE__, __LINE__, RESET, RED, RESET);
  
  
  // std::string STRING_IP_CNTRL;
  // std::string STRING_SYS_ID;
  
  // if (argc < 3)
  // {
  //   printf( " [ %s%s:%d%s ]\t %s ERROR: invalid input argument!%s \n", GREEN, __FUNCFILE__, __LINE__, RESET, RED, RESET);
  //   return -1;
  // } 
  // else
  // {
  //   STRING_IP_CNTRL = argv[1];
  //   STRING_SYS_ID   = argv[2];
  //   printf( " [ %s%s:%d%s ]\t Connection to %s: %s.c5g\n", GREEN, __FUNCFILE__, __LINE__, RESET, STRING_IP_CNTRL.c_str(), STRING_SYS_ID.c_str() );
  // }
  
  // ros::init( argc, argv, "c5gopen_controller_node" );
  // ros::NodeHandle nh;
  
  // ros::Rate loop_rate(100);
        
  // int period = ORL_0_4_MILLIS;
  
  // cycle_active = true;
  // printf("CYCLE Enabled\n");
  
  // system_initialized = false;
  
  // std::string trj_namespace = "/comau_c5gopen_trj"; 
  // C5gopenTrjExec **c5gopen_exec = new C5gopenTrjExec*[MAX_NUM_ARMS];
  
   
  // for( int iArm=0; iArm<MAX_NUM_ARMS; iArm++ )
  // {
  //   flag_RunningMove            [iArm] = false;
  //   flag_ExitFromOpen           [iArm] = false;
  //   modality_active             [iArm] = CRCOPEN_LISTEN;
  //   modality_old                [iArm] = CRCOPEN_LISTEN;
  //   robot_movement_enabled      [iArm] = false;
  //   trajectory_in_execution     [iArm] = false;
  //   received_delta_target_cart  [iArm] = false;
    
  //   memset( &actual_joints_position[iArm], 0x00, sizeof(ORL_joint_value));
  //   memset( &actual_cartesian_position[iArm], 0x00, sizeof(ORL_cartesian_position));

  //   delta_target_cart_pos[iArm] = new itia::butils::circ_buffer<delta_position_t>(delta_target_cart_pos_max_len);
  //   absolute_target_position[iArm] = new itia::butils::circ_buffer<absolute_target_position_t>(target_pos_max_buff_len);
  //   absolute_target_position_log[iArm] = new itia::butils::circ_buffer<absolute_target_position_t>(target_pos_max_buff_log_len);
    
  //   c5gopen_exec[iArm] = new C5gopenTrjExec( iArm+1, period, trj_namespace, &nh, delta_target_cart_pos[iArm], absolute_target_position[iArm] );
    
  // }
   
  // mask_moving_arms = 0;

  // if( ORLOPEN_initialize_controller ( STRING_IP_CNTRL.c_str(), STRING_SYS_ID.c_str(), ORL_SILENT, ORL_CNTRL01) != ORLOPEN_RES_OK ) 
  // {
  //   ORLOPEN_initialize_controller ( STRING_IP_CNTRL.c_str(), STRING_SYS_ID.c_str(), ORL_VERBOSE, ORL_CNTRL01);
  //   printf( " [ %s%s:%d%s ]\t %serror in ORL_initialize_controller%s\n", GREEN, __FUNCFILE__, __LINE__, RESET, RED, RESET );
  //   exit(0);
  // }
  // else 
  //     printf( " [ %s%s:%d%s ]\t %s: %s.c5g OK\n", GREEN, __FUNCFILE__, __LINE__, RESET, STRING_IP_CNTRL.c_str(), STRING_SYS_ID.c_str() );

  
  // if ( ORLOPEN_set_period( period, ORL_SILENT, ORL_CNTRL01 ) != ORLOPEN_RES_OK )
  // {
  //   ORLOPEN_set_period( period, ORL_VERBOSE, ORL_CNTRL01 );
  //   printf( " [ %s%s:%d%s ]\t %serror in ORLOPEN_set_period%s\n", GREEN, __FUNCFILE__, __LINE__, RESET, RED, RESET );
  //   exit(0);
  // }
  
  // ORL_cartesian_position bFrame, tFrame, uFrame;
  // if ( !set_frames( &nh, &bFrame, &tFrame, &uFrame ) )
  // {
  //   printf( " [ %s%s:%d%s ]\t %serror cannot set frames%s\n", GREEN, __FUNCFILE__, __LINE__, RESET, RED, RESET );
  //   exit(0);
  // }
  
  // if ( ORL_initialize_frames( bFrame, tFrame, uFrame, ORL_SILENT, ORL_CNTRL01, ORL_ARM1 ) != ORLOPEN_RES_OK )  
  // {
  //   ORL_initialize_frames( bFrame, tFrame, uFrame, ORL_VERBOSE, ORL_CNTRL01, ORL_ARM1 );
  //   printf( " [ %s%s:%d%s ]\t %serror in ORL_initialize_frames%s\n", GREEN, __FUNCFILE__, __LINE__, RESET, RED, RESET );
  //   exit(0);
  // } 
    
  // if ( ORLOPEN_SetCallBackFunction( &user_callback, ORL_SILENT, ORL_CNTRL01 ) < ORLOPEN_RES_OK )
  // {
  //   ORLOPEN_SetCallBackFunction( &user_callback, ORL_VERBOSE, ORL_CNTRL01 );
  //   printf( " [ %s%s:%d%s ]\t %serror in ORLOPEN_SetCallBackFunction%s\n", GREEN, __FUNCFILE__, __LINE__, RESET, RED, RESET );
  //   exit(0);
  // }
  // else
  //   printf( " [ %s%s:%d%s ]\t user callback function initialized \n", GREEN, __FUNCFILE__, __LINE__, RESET );  
  
  // ros::Duration(1).sleep();
  
  // /******************** Start with C5Gopen control *******************/

  // if ( ORLOPEN_StartCommunication( ORL_SILENT ) != ORLOPEN_RES_OK )
  // {
  //   ORLOPEN_StartCommunication( ORL_VERBOSE );
  //   ORLOPEN_GetPowerlinkState( ORL_VERBOSE );
  //   exit(0);
  // }
  
  // ros::Duration(2).sleep();
      
  // if ( initialize_control_position() < 0 ) 
  // {
  //   printf( " [ %s%s:%d%s ]\t %serror in initialize_control_position() function %s\n", GREEN, __FUNCFILE__, __LINE__, RESET, RED, RESET );
  //   exit(0);
  // }
  
  // /******************** Start parallel thread *******************/
  
  // pthread_t  loop_console_thread_id, publisher_thread_id;
  
  // try
  // {
  //   UserThreadSharedStruct shared_with_thread( period, &nh );
    
  //   struct sched_param loop_console_thread_param, publisher_thread_param;
    
  //   // Publisher thread
  //   printf( " [ %s%s:%d%s ]\t entering in the publisher thread... \n", GREEN, __FUNCFILE__, __LINE__, RESET ); 
  //   int publisher_thread_rc = pthread_create ( &publisher_thread_id, 
  //                                               NULL, 
  //                                               publisher_thread, 
  //                                               (void*) &shared_with_thread);

  //   if ( publisher_thread_rc  < 0 )
  //     throw std::invalid_argument( " [ %s%s:%d%s ] failed @ pthread_create(publisher_thread_rc)" );
    
    
  //   memset(&publisher_thread_param, 0x0, sizeof(sched_param));
  //   publisher_thread_param.sched_priority = min_prio_for_policy;
    
  //   if ( pthread_setschedparam( publisher_thread_id, policy, &publisher_thread_param ) != 0 )
  //     printf( " [ %s%s:%d%s ]\t %s ERROR in pthread_setschedparam() of publisher_thread_id%s \n", GREEN, __FUNCFILE__, __LINE__, RESET, RED, RESET);

  //   pthread_setname_np( publisher_thread_id, "c5gopen_ext_publisher" );

    
    
  //   // Loop Console thread
  //   printf( " [ %s%s:%d%s ]\t entering in the loop_console thread... \n", GREEN, __FUNCFILE__, __LINE__, RESET ); 
  //   int loop_console_thread_rc = pthread_create ( &loop_console_thread_id, 
  //                                                 NULL, 
  //                                                 loop_console_thread, 
  //                                                 (void*) &shared_with_thread );

  //   if ( loop_console_thread_rc  < 0 )
  //     throw std::invalid_argument( " [ %s%s:%d%s ] failed @ pthread_create(loop_console_thread)" );
    
        
  //   memset(&loop_console_thread_param, 0x0, sizeof(sched_param));
  //   loop_console_thread_param.sched_priority = min_prio_for_policy;
    
  //   if ( pthread_setschedparam( loop_console_thread_id, policy, &loop_console_thread_param ) != 0 )
  //     printf( " [ %s%s:%d%s ]\t %s ERROR in pthread_setschedparam() of loop_console_thread_id %s \n", GREEN, __FUNCFILE__, __LINE__, RESET, RED, RESET);

  //   pthread_setname_np( loop_console_thread_id, "c5gopen_loop_console" );
    
    
  //   /// Verify if the scheduler is properly set   
  //   if ( pthread_getschedparam(pthread_self(), &policy, &publisher_thread_param) != 0 )
  //     printf( " [ %s%s:%d%s ]\t %s ERROR in pthread_getschedparam() for publisher_thread_id %s \n", GREEN, __FUNCFILE__, __LINE__, RESET, RED, RESET);
    
  //   printf( " [ %s%s:%d%s ]\t %s main_thread policy = %s \t priority = %d %s \n", GREEN, __FUNCFILE__, __LINE__, RESET, BOLDGREEN, 
  //                                                                               ( policy == SCHED_OTHER ? "SCHED_OTHER" : ( policy == SCHED_FIFO ? "SCHED_FIFO": ( policy == SCHED_RR ? "SCHED_RR": "UNHANDELED POLICY" ) ) ) , 
  //                                                                                 min_prio_for_policy, RESET ); 
    
  //   if ( pthread_getschedparam(publisher_thread_id, &policy, &publisher_thread_param) != 0 )
  //     printf( " [ %s%s:%d%s ]\t %s ERROR in pthread_getschedparam() for publisher_thread_id %s \n", GREEN, __FUNCFILE__, __LINE__, RESET, RED, RESET);
    
  //   printf( " [ %s%s:%d%s ]\t %s publisher_thread_id policy = %s \t priority = %d %s \n", GREEN, __FUNCFILE__, __LINE__, RESET, BOLDGREEN, 
  //                                                                                       ( policy == SCHED_OTHER ? "SCHED_OTHER" : ( policy == SCHED_FIFO ? "SCHED_FIFO": ( policy == SCHED_RR ? "SCHED_RR": "UNHANDELED POLICY" ) ) ) , 
  //                                                                                         min_prio_for_policy, RESET );
    
  //   if ( pthread_getschedparam(loop_console_thread_id, &policy, &loop_console_thread_param) != 0 ) 
  //     printf( " [ %s%s:%d%s ]\t %s ERROR in pthread_getschedparam() for loop_console_thread_id %s \n", GREEN, __FUNCFILE__, __LINE__, RESET, RED, RESET);
    
  //   printf( " [ %s%s:%d%s ]\t %s loop_console_thread_id policy = %s \t priority = %d %s \n", GREEN, __FUNCFILE__, __LINE__, RESET, BOLDGREEN, 
  //                                                                                          ( policy == SCHED_OTHER ? "SCHED_OTHER" : ( policy == SCHED_FIFO ? "SCHED_FIFO": ( policy == SCHED_RR ? "SCHED_RR": "UNHANDELED POLICY" ) ) ) , 
  //                                                                                            min_prio_for_policy, RESET );
    
  // }
  // catch ( std::invalid_argument& e )
  // {
  //   printf( " [ %s%s:%d%s ] invalid argument %s \n", RED, __FUNCFILE__, __LINE__, RESET, e.what() );
  //   return -1;
  // }
  // catch (...)
  // {
  //   printf( " [ %s%s:%d%s ] unhandled exception!\n", RED, __FUNCFILE__, __LINE__, RESET);
  //   return -1;
  // }
   
  // while( ros::ok() && !std::all_of( std::begin(flag_ExitFromOpen), std::end(flag_ExitFromOpen), [](bool i) { return i; } ) )
  // {
  //   for (int iArm=0; iArm<MAX_NUM_ARMS; iArm++)
  //   {
  //     if ( !flag_ExitFromOpen[iArm] )
  //     { 
  //       trajectory_in_execution[iArm]     = c5gopen_exec[iArm]->getTrjExecutionFlag();
  //       robot_movement_enabled[iArm]      = c5gopen_exec[iArm]->getRobMoveEnabledFlag();
  //       received_delta_target_cart[iArm]  = c5gopen_exec[iArm]->getRcvDeltaTargetCartFlag(); 
  //     }
  //     else
  //     {
  //       trajectory_in_execution[iArm]     = false;
  //       robot_movement_enabled[iArm]      = false;
  //       received_delta_target_cart[iArm]  = false;
  //     }
  //   }
  //   ros::spinOnce();
  //   loop_rate.sleep();
  // }
  
  // ros::Duration(2).sleep();
  
  // ORLOPEN_StopCommunication( ORL_VERBOSE );
  
  // ros::Duration(4).sleep();
  
  // ORL_terminate_controller( ORL_VERBOSE, ORL_CNTRL01 );
   
  // for( int iArm=0; iArm<MAX_NUM_ARMS; iArm++ )
  // {
  //   delete delta_target_cart_pos[iArm];
  //   delete absolute_target_position[iArm];
  //   delete absolute_target_position_log[iArm];
  //   delete c5gopen_exec[iArm];
  // } 
  
  // delete [] c5gopen_exec;

  // pthread_cancel(publisher_thread_id);  
  // pthread_cancel(loop_console_thread_id);

  // ros::Duration(1).sleep();
  
  return 0;
}

