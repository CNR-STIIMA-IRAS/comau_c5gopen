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

#include <fstream>

#include <cnr_logger/cnr_logger.h>
#include <cnr_logger/cnr_logger_macros.h>

#include <comau_c5gopen_lpc/c5gopen_driver.h>
#include <comau_c5gopen_lpc/c5gopen_utilities.h>
#include <comau_c5gopen_lpc/mqtt.h>

std::unique_ptr<c5gopen::C5GOpenDriver> c5gopen_driver;

void on_publish_callback()
{
  for ( const size_t& active_arm : c5gopen_cfg.c5gopen_driver_cfg_.active_arms_ )
  {
    c5gopen::RobotJointState joint_state_link = c5gopen_driver->getRobotJointStateLink( active_arm );
    c5gopen::RobotJointState joint_state_motor = c5gopen_driver->getRobotJointStateMotor( active_arm );
    c5gopen::RobotCartState cart_state = c5gopen_driver->getRobotCartState( active_arm );
    ORL_joint_value motor_current = c5gopen_driver->getRobotMotorCurrent( active_arm );
  }

}

void on_subscribe_callback()
{
  for ( const size_t& active_arm : c5gopen_cfg.c5gopen_driver_cfg_.active_arms_ )
  {
    // if ( !setRobotJointAbsoluteTargetPosition( active_arm, const RobotJointState& joint_state ))
    //   CNR_WARN("Can't add new trajectory point, the C5GOpenDriver buffer is full.");
  }
}

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

  try
  {

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
    // Create the MQTT client
    // ************************************************
    std::unique_ptr<cnr::mqtt_client> iot_client( new cnr::mqtt_client( c5gopen_cfg.mqtt_cfg_.mqtt_client_id_.c_str(),
                                                                        c5gopen_cfg.mqtt_cfg_.mqtt_broker_address_.c_str(),
                                                                        c5gopen_cfg.mqtt_cfg_.mqtt_port_) ) ;

    for (const std::string& topic: c5gopen_cfg.mqtt_cfg_.mqtt_sub_topics_)
      iot_client->subscribe( NULL, topic.c_str(), 1 );

    iot_client->on_publish(&on_publish_callback,NULL);
    iot_client->on_subscribe(&on_subscribe_callback,NULL);

    // ************************************************
    // Create logger 
    // ************************************************
    std::shared_ptr<cnr_logger::TraceLogger> logger( new cnr_logger::TraceLogger( "c5gopen", c5gopen_cfg.cnr_logger_cfg_file, true, true));  


    // ************************************************
    // Create, initialize and run C5GOpen driver  
    // ************************************************
    c5gopen_driver.reset( new c5gopen::C5GOpenDriver( c5gopen_cfg.c5gopen_driver_cfg_, logger) );

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
        int rc = iot_client->loop();
        if (rc)
          //iot_client->reconnect();
        else
          //iot_client->subscribe(NULL, MQTT_TOPIC);  
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

