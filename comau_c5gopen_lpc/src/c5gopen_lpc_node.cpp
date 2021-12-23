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

#include <cnr_logger/cnr_logger.h>
#include <cnr_logger/cnr_logger_macros.h>

#include <comau_c5gopen_lpc/c5gopen_driver.h>

c5gopen::CallbackBase* c5gopen::AvailableCallbackSlots[1];

int  main (int argc, char **argv)
{
  std::string ip_ctrl;
  std::string sys_id;
  std::string logger_cfg_name;
  
  if( argc < 5) 
  {
    ip_ctrl = std::string(argv[1]);
    sys_id  = std::string(argv[2]);
    logger_cfg_name = std::string(argv[3]);    
  }
  else
  {
    std::cout << cnr_logger::RED() <<  "ERROR: wrong number of input to the c5gopen LPC node. The right order is: COMAU ROBOT IP, COMAU SYSTEM ID, LOGGER CONFIG PATH" << cnr_logger::RESET() << std::endl;
    return -1;
  }

  // Create logger object
  std::shared_ptr<cnr_logger::TraceLogger> logger(new cnr_logger::TraceLogger("c5gopen", 
                                                                              logger_cfg_name, 
                                                                              true, 
                                                                              true));  
 
  int c5gopen_period = ORL_0_4_MILLIS;
  
  // /******************** Start parallel thread *******************/  
  try
  {
    c5gopen::C5GOpenDriver c5gopen_driver(ip_ctrl,sys_id,c5gopen_period,logger);

    c5gopen::AvailableCallbackSlots[0] = new c5gopen::DynamicCallback<0x00>();
    c5gopen::CF callback_comau; 

    callback_comau = c5gopen::MemberFunctionCallback(&c5gopen_driver, &c5gopen::C5GOpenDriver::c5gopen_callback);

    // int ret = (*callback_comau)(0);
    // cout << "return: " << ret;

    c5gopen_driver.init();

    c5gopen_driver.run();

    while(c5gopen_driver.getThreadsStatus())
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }
  catch ( std::invalid_argument& e )
  {
    CNR_ERROR( *logger, "Invalid argument " + std::string(e.what())  );
    return -1;
  }
  catch (...)
  {
    CNR_ERROR( *logger, "Unhandled exception!");
    return -1;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  
  return 0;
}

