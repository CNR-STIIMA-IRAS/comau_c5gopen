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

/* author Enrico Villagrossi (enrico.villagrossi@stiima.cnr.it) */

#include <arpa/inet.h>
#include <sys/socket.h>
#include <netdb.h>
#include <ifaddrs.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/if_link.h>


#include <comau_c5gopen_lpc/realtime_utilities.h>

namespace realtime_utilities
{


  bool setprio(int prio, int sched)
  {
    struct sched_param param;
    // Set realtime priority for this thread
    param.sched_priority = prio;
    if (sched_setscheduler(0, sched, &param) < 0)
    {
      //ROS_ERROR("sched_setscheduler");
      return false;
    }
    return true;
  }

  bool show_new_pagefault_count(const char* logtext, const char* allowed_maj, const char* allowed_min)
  {
    static int last_majflt = 0, last_minflt = 0;
    struct rusage usage;

    getrusage(RUSAGE_SELF, &usage);

    printf("%-30.30s: Pagefaults, Major:%ld (Allowed %s), " \
          "Minor:%ld (Allowed %s)\n", logtext,
          usage.ru_majflt - last_majflt, allowed_maj,
          usage.ru_minflt - last_minflt, allowed_min);

    last_majflt = usage.ru_majflt;
    last_minflt = usage.ru_minflt;
    return true;
  }

  bool prove_thread_stack_use_is_safe(size_t  stacksize)
  {
    volatile char buffer[stacksize];
    size_t i;

    /* Prove that this thread is behaving well */
    for (i = 0; i < stacksize; i += sysconf(_SC_PAGESIZE))
    {
      /* Each write to this buffer shall NOT generate a
        pagefault. */
      buffer[i] = i;
    }

    return show_new_pagefault_count("Caused by using thread stack", "0", "0");
  }

  /*************************************************************/

  bool error(int at)
  {
    fprintf(stderr, "Some error occured at %d", at);
    exit(1);
    return false;
  }


  bool configure_malloc_behavior(void)
  {
    /* Now lock all current and future pages from preventing of being paged */
    if (mlockall(MCL_CURRENT | MCL_FUTURE))
    {
      //ROS_ERROR("mlockall failed:");
      switch (errno)
      {
      case ENOSYS:
        std::cerr << "The implementation does not support this memory locking interface." << std::endl;
        break;
      case EAGAIN:
        std::cerr << "Some or all of the memory identified by the operation could not be locked when the call was made." << std::endl;
        break;
      case EINVAL:
        std::cerr << "The flags argument is zero, or includes unimplemented flags." << std::endl;
        break;
      case ENOMEM:
        std::cerr << "Locking all of the pages currently mapped into the address space of the process would exceed an implementation-dependent limit on the amount of memory that the process may lock." << std::endl;
        break;
      case EPERM:
        std::cerr << "The calling process does not have the appropriate privilege to perform the requested operation." << std::endl;
        break;
      }
      return false;
    }


    /* Turn off malloc trimming.*/
    mallopt(M_TRIM_THRESHOLD, -1);

    /* Turn off mmap usage. */
    mallopt(M_MMAP_MAX, 0);

    return true;
  }

  bool reserve_process_memory(size_t size)
  {
    size_t i;
    char *buffer;

    buffer = (char*)malloc(size);

    /* Touch each page in this piece of memory to get it mapped into RAM */
    for (i = 0; i < size; i += sysconf(_SC_PAGESIZE))
    {
      /* Each write to this buffer will generate a pagefault.
          Once the pagefault is handled a page will be locked in
          memory and never given back to the system. */
      buffer[i] = 0;
    }

    /* buffer will now be released. As Glibc is configured such that it
        never gives back memory to the kernel, the memory allocated above is
        locked for this process. All malloc() and new() calls come from
        the memory pool reserved and locked above. Issuing free() and
        delete() does NOT make this locking undone. So, with this locking
        mechanism we can build C++ applications that will never run into
        a major/minor pagefault, even with swapping enabled. */
    free(buffer);

    return true;
  }


  bool rt_main_init(size_t pre_allocation_size)
  {
    if (!show_new_pagefault_count("Initial count", ">=0", ">=0"))
    {
      printf("Error in show_new_pagefault_count. Exit.");
      return false;
    }

    if (!configure_malloc_behavior())
    {
      printf("Error in configure_malloc_behavior. Exit.");
      return false;
    }


    if (!show_new_pagefault_count("mlockall() generated", ">=0", ">=0"))
    {
      printf("Error in show_new_pagefault_count. Exit.");
      return false;
    }


    if (!reserve_process_memory(pre_allocation_size))
    {
      printf("Error in reserve_process_memory. Exit.");
      return false;
    }


    if (!show_new_pagefault_count("malloc() and touch generated", ">=0", ">=0"))
    {
      printf("Error in show_new_pagefault_count. Exit.");
      return false;
    }


    /* Now allocate the memory for the 2nd time and prove the number of pagefaults are zero */
    if (!reserve_process_memory(pre_allocation_size))
    {
      printf("Error in reserve_process_memory. Exit.");
      return false;
    }

    if (!show_new_pagefault_count("2nd malloc() and use generated", "0", "0"))
    {
      printf("Error in show_new_pagefault_count. Exit.");
      return false;
    }

    printf("\n\nLook at the output of ps -leyf, and see that the RSS is now about %zu [MB]\n", pre_allocation_size / (1024 * 1024));

    return true;
  }


  uint32_t timer_inc_period(period_info *pinfo)
  {
    int missed_deadlines = -1;
    struct timespec act_time;
    clock_gettime(CLOCK_MONOTONIC, &act_time);

    double at = act_time.tv_sec + double(act_time.tv_nsec) / 1.e9;
    double nt = 0;

    do
    {
      missed_deadlines++;
      pinfo->next_period.tv_nsec += pinfo->period_ns;

      while (pinfo->next_period.tv_nsec >= 1000000000)
      {
        /* timespec nsec overflow */
        pinfo->next_period.tv_sec++;
        pinfo->next_period.tv_nsec -= 1000000000;
      }
      nt = pinfo->next_period.tv_sec + double(pinfo->next_period.tv_nsec) / 1.e9;

    }
    while (nt < at);

    return missed_deadlines;
  }

  uint32_t timer_inc_period(period_info *pinfo, int64_t offest_time_ns)
  {
    struct timespec before;
    before.tv_nsec = pinfo->next_period.tv_nsec;
    before.tv_sec  = pinfo->next_period.tv_sec;
    timer_add(&pinfo->next_period, pinfo->period_ns + offest_time_ns);

    double d = timer_difference_s(&before, &(pinfo->next_period));
    return d > 0 ? uint64_t(d * 1e9) / pinfo->period_ns : 0;
  }


  void timer_periodic_init(period_info *pinfo, long period_ns)
  {
    pinfo->period_ns = period_ns;
    clock_gettime(CLOCK_MONOTONIC, &(pinfo->next_period));
  }

  int timer_wait_rest_of_period(struct timespec *ts)
  {
    // int ret = timer_inc_period(pinfo);

    /* for simplicity, ignoring possibilities of signal wakes */
    int err = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, ts, NULL);
    switch (err)
    {
    case EFAULT :
      printf("request or remain specified an invalid address.\n");
      break;
    case EINTR  :
      printf("The sleep was interrupted by a signal handler; see signal(7)\n");
      break;
    case EINVAL :
      printf("The value in the tv_nsec field was not in the range 0 to 999999999 or tv_sec was negative.\n");
      break;
    case ENOTSUP:
      printf("clock_id was invalid.  (CLOCK_THREAD_CPUTIME_ID is not a supported");
      break;
    }
    return err;
  }


  void timer_add(struct timespec *ts, int64_t addtime)
  {
    int64_t sec, nsec;

    nsec = addtime % 1000000000;
    sec = (addtime - nsec) / 1000000000;
    ts->tv_sec += sec;
    ts->tv_nsec += nsec;
    if (ts->tv_nsec > 1000000000)
    {
      nsec = ts->tv_nsec % 1000000000;
      ts->tv_sec += (ts->tv_nsec - nsec) / 1000000000;
      ts->tv_nsec = nsec;
    }
  }

  void timer_calc_sync_offset(int64_t reftime, int64_t cycletime, int64_t *offsettime)
  {
    static int64_t integral = 0;
    int64_t        delta;

    /* set linux sync point 50us later than DC sync, just as example */
    delta = (reftime - 50000) % cycletime;
    if (delta > (cycletime / 2))
    {
      delta = delta - cycletime;
    }
    if (delta > 0)
    {
      integral++;
    }
    if (delta < 0)
    {
      integral--;
    }
    *offsettime = -(delta / 100) - (integral / 20);
  }

  int64_t timer_difference_ns(struct timespec const * timeA_p, struct timespec const *timeB_p)
  {
    int64_t ret = timeA_p->tv_nsec - timeB_p->tv_nsec;
    ret +=  1e9*( timeA_p->tv_sec  - timeB_p->tv_sec );
    return ret;
  }


  double timer_difference_s(struct timespec const * timeA_p, struct timespec const *timeB_p)
  {
    double ret = static_cast<double>( timeA_p->tv_nsec - timeB_p->tv_nsec ) / (double(1e9));
    ret       += static_cast<double>( timeA_p->tv_sec  - timeB_p->tv_sec );
    return ret;
  }


  bool timer_greater_than(struct timespec const * timeA_p, struct timespec const *timeB_p)
  {
    if( timeA_p->tv_sec > timeB_p->tv_sec)
      return true;
    else if( timeA_p->tv_sec == timeB_p->tv_sec)
      return timeA_p->tv_nsec > timeB_p->tv_nsec;
    else
      return false;
  }

  double timer_to_s(const timespec*timeA_p)
  {
    return ((double)(timeA_p->tv_sec) + ((double) timeA_p->tv_nsec) / 1.e9);
  }

  int64_t timer_to_ns(const struct timespec *timeA_p)
  {
    return (1e9*timeA_p->tv_sec + timeA_p->tv_nsec );
  }


  std::vector<std::string> get_ifaces()
  {
    std::vector<std::string> ret;
    ifaddrs *ifaddr, *ifa;
    int family, s, n;
    char host[NI_MAXHOST];

    if (getifaddrs(&ifaddr) == -1)
    {
      //ROS_ERROR("getifaddrs");
      return ret;
    }

    /* Walk through linked list, maintaining head pointer so we
      can free list later */

    for (ifa = ifaddr, n = 0; ifa != NULL; ifa = ifa->ifa_next, n++)
    {
      if (ifa->ifa_addr == NULL)
        continue;

      family = ifa->ifa_addr->sa_family;

      /* Display interface name and family (including symbolic
        form of the latter for the common families) */

      printf("%-8s %s (%d)\n",
            ifa->ifa_name,
            (family == AF_PACKET) ? "AF_PACKET" :
            (family == AF_INET) ? "AF_INET" :
            (family == AF_INET6) ? "AF_INET6" : "???",
            family);

      /* For an AF_INET* interface address, display the address */

      if (family == AF_INET || family == AF_INET6)
      {
        s = getnameinfo(ifa->ifa_addr,
                        (family == AF_INET) ? sizeof(struct sockaddr_in) :
                        sizeof(struct sockaddr_in6),
                        host, NI_MAXHOST,
                        NULL, 0, NI_NUMERICHOST);
        if (s != 0)
        {
          printf("getnameinfo() failed: %s\n", gai_strerror(s));
          exit(EXIT_FAILURE);
        }

        printf("\t\taddress: <%s>\n", host);

      }
      else if (family == AF_PACKET && ifa->ifa_data != NULL)
      {
        rtnl_link_stats *stats = (rtnl_link_stats*) ifa->ifa_data;

        printf("\t\ttx_packets = %10u; rx_packets = %10u\n"
              "\t\ttx_bytes   = %10u; rx_bytes   = %10u\n",
              stats->tx_packets, stats->rx_packets,
              stats->tx_bytes, stats->rx_bytes);
      }
      ret.push_back(ifa->ifa_name);
    }

    freeifaddrs(ifaddr);
    return ret;
  }
}
