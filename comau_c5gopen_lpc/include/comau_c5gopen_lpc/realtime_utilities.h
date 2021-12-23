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

#ifndef __CNR__RT__UTILS__H__
#define __CNR__RT__UTILS__H__


#include <stdlib.h>
#include <stdio.h>
#include <sys/mman.h>  // Needed for mlockall()
#include <unistd.h>    // needed for sysconf(int name);
#include <malloc.h>
#include <sys/time.h>  // needed for getrusage
#include <sys/resource.h>  // needed for getrusage
#include <pthread.h>
#include <limits.h>
#if defined(__COBALT__) && !defined(__COBALT_WRAP__)
#include <alchemy/task.h>
#endif
#include <iostream>
#include <ctime>
#include <cassert>
#include <ratio>
#include <chrono>
#include <mutex>
#include <string>
#include <vector>

namespace realtime_utilities
{
  struct period_info
  {
    struct timespec next_period;
    long period_ns;
  };

  bool setprio(int prio, int sched);

  bool show_new_pagefault_count(const char* logtext, const char* allowed_maj, const char* allowed_min);

  bool prove_thread_stack_use_is_safe(size_t stacksize);

  bool error(int at);

  bool configure_malloc_behavior(void);

  bool reserve_process_memory(size_t size);

  bool rt_main_init(size_t pre_allocation_size);

  uint32_t timer_inc_period(period_info *pinfo);

  uint32_t timer_inc_period(period_info *pinfo, int64_t offest_time);

  void   timer_periodic_init(period_info *pinfo, long period_ns);

  int    timer_wait_rest_of_period(struct timespec *ts);

  void   timer_add(struct timespec *ts, int64_t addtime);


  void    timer_calc_sync_offset(int64_t reftime, int64_t cycletime, int64_t *offsettime);
  double  timer_difference_s(struct timespec const * timeA_p, struct timespec const *timeB_p);
  int64_t timer_difference_ns(struct timespec const * timeA_p, struct timespec const *timeB_p);
  bool    timer_greater_than(struct timespec const * timeA_p, struct timespec const *timeB_p);
  double  timer_to_s(const struct timespec *timeA_p);
  int64_t timer_to_ns(const struct timespec *timeA_p);

  std::vector<std::string> get_ifaces();


  inline
  bool rt_init_thread(size_t stack_size, int prio, int sched, period_info*  pinfo, long  period_ns)
  {
  #if defined(__COBALT__) && !defined(__COBALT_WRAP__)
    RT_TASK *curtask;
    RT_TASK_INFO curtaskinfo;
    curtask = rt_task_self();
    int r = rt_task_inquire(curtask, &curtaskinfo);
    if (r != 0)
    {
      switch (r)
      {
      case EINVAL:
      case -EINVAL:
        printf("task is not a valid task descriptor, or if prio is invalid.");
        break;
      case EPERM :
      case -EPERM :
        printf("task is NULL and this service was called from an invalid context.");
        break;
      }
      return false;
    }

    r = rt_task_set_priority(NULL, prio);
    if (r != 0)
    {
      switch (r)
      {
      case EINVAL:
      case -EINVAL:
        printf("task is not a valid task descriptor, or if prio is invalid.");
        break;
      case EPERM :
      case -EPERM :
        printf("task is NULL and this service was called from an invalid context.");
        break;
      }
      return false;
    }

    //Make the task periodic with a specified loop period
    rt_task_set_periodic(NULL, TM_NOW, period_ns);
  #else

    if (!setprio(prio, sched))
    {
      printf("Error in setprio.");
      return false;
    }

    printf("I am an RT-thread with a stack that does not generate page-faults during use, stacksize=%zu\n", stack_size);

    //<do your RT-thing here>

    show_new_pagefault_count("Caused by creating thread", ">=0", ">=0");
    prove_thread_stack_use_is_safe(stack_size);
  #endif

    if (pinfo != NULL)
    {
      assert(period_ns > 0);
      timer_periodic_init(pinfo, period_ns);
    }

    return true;
}


}

#endif