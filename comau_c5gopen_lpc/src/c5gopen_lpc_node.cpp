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

#include <fstream>

#include <cnr_logger/cnr_logger.h>
#include <cnr_logger/cnr_logger_macros.h>

#include <comau_c5gopen_lpc/c5gopen_mqtt.h>
#include <comau_c5gopen_lpc/c5gopen_driver.h>
#include <comau_c5gopen_lpc/c5gopen_utilities.h>

int  main (int argc, char **argv)
{
  std::string c5gopen_cfg_file_name;
  if( argc == 2) 
    c5gopen_cfg_file_name = std::string(argv[1]);
  else
  {
    std::cout << cnr_logger::RED() <<  "Error: wrong number of input to the c5gopen LPC node. The right order is: C5GOPEN_CONFIG_FILE_NAME" << cnr_logger::RESET() << std::endl;
    return -1;
  }

  // ************************************************
  // Load configuration parameters
  // ************************************************
  c5gopen::C5GOpenNodeCfg c5gopen_cfg;
  if (!c5gopen::load_c5gopen_parameters(c5gopen_cfg_file_name, c5gopen_cfg))
  {
    std::cout << cnr_logger::RED() <<  "Error: cannot load the configuration parameters, please check the configuration file: " << c5gopen_cfg_file_name << cnr_logger::RESET() << std::endl;
    return -1;
  }

  // ************************************************
  // Create logger 
  // ************************************************
  std::shared_ptr<cnr_logger::TraceLogger> logger( new cnr_logger::TraceLogger( "c5gopen", c5gopen_cfg.cnr_logger_cfg_file, true, true));  

  try
  {
    // ************************************************
    // Create the MQTT client
    // ************************************************
    std::unique_ptr<c5gopen::C5GOpenMQTT> iot_client( new c5gopen::C5GOpenMQTT( c5gopen_cfg.mqtt_cfg_.mqtt_client_id_.c_str(),
                                                                                c5gopen_cfg.mqtt_cfg_.mqtt_broker_address_.c_str(),
                                                                                c5gopen_cfg.mqtt_cfg_.mqtt_port_,
                                                                                logger) ) ;

    for (const std::string& topic: c5gopen_cfg.mqtt_cfg_.mqtt_sub_topics_)
      iot_client->subscribe( NULL, topic.c_str(), 1 );

    // ************************************************
    // Create, initialize and run C5GOpen driver  
    // ************************************************
    std::shared_ptr<c5gopen::C5GOpenDriver> c5gopen_driver( new c5gopen::C5GOpenDriver( c5gopen_cfg.c5gopen_driver_cfg_, logger ));

//TO DEBUG!!!!!!

    while(1)
    {
      if(iot_client)
      {
        iot_client->publish( c5gopen_driver );   
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));

    }

    if ( !c5gopen_driver->init() ) 
    {
      CNR_ERROR( *logger, "Unable to init C5GOpen");
      return -1;
    }
      
    if ( !c5gopen_driver->run() )
    {
      CNR_ERROR( *logger, "Unable to run C5GOpen");
      return -1;
    }

    // ************************************************
    // Enter in the infinite loop  
    // ************************************************
    while(c5gopen_driver->getC5GOpenThreadsStatus() != c5gopen::thread_status::CLOSED ||
          c5gopen_driver->getComThreadsStatus() != c5gopen::thread_status::CLOSED || 
          c5gopen_driver->getLoopConsoleThreadsStatus() != c5gopen::thread_status::CLOSED )
    {
      if(iot_client)
      {
        iot_client->publish( c5gopen_driver );   
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(1));
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

  CNR_INFO(*logger, "Closing the main c5gopen node");
  
  return 0;
}

