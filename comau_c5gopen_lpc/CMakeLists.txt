cmake_minimum_required(VERSION 2.8.3)
project(comau_c5gopen_lpc)

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -pg -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 ")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--copy-dt-needed-entries")

option(USE_C5GOPEN "USE C5GOPEN" ON)
if(NOT USE_C5GOPEN)
  message(-- " disable C5GOPEN")
  add_definitions(-DC5GOPEN_NOT_ENABLED)
endif(NOT USE_C5GOPEN)

execute_process(COMMAND uname -r OUTPUT_VARIABLE UNAME_RESULT OUTPUT_STRIP_TRAILING_WHITESPACE)
message(-- " Kernel version: " ${UNAME_RESULT})
if(UNAME_RESULT MATCHES "^([^rt]*)rt")
  message(-- " using realtime")
  set(PREEMPTIVE_RT ON CACHE BOOL "Compile library for preemptive machines")
else()
  message(-- " using NO realtime")
  set(PREEMPTIVE_RT OFF CACHE BOOL "Compile library for NON preemptive machines")
endif()

### Boost
find_package( Boost REQUIRED COMPONENTS system filesystem thread)

### YAML-CPP
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

### Mosquitto
set(Mosquitto_LIBRARIES mosquitto)

include_directories(include
                    ${Boost_INCLUDE_DIRS}
                    ${YAML_CPP_INCLUDE_DIRS}
                    /opt/inc/
                    )

link_directories( ${Boost_LIBRARY_DIR} 
                  ${YAML_CPP_LIBRARY_DIR} )                


if(EXISTS "/opt/inc/eORL.h")
  MESSAGE(-- " eORL AVAILABLE")
  option(EORL_AVAILABLE "eORL library AVAILABLE" ON)
  add_definitions(-DEORL_AVAILABLE)  
 else()
  MESSAGE(-- " eORL NOT ENABLED")
  option(EORL_AVAILABLE "eORL library NOT AVAILABLE" OFF)
  message(WARNING "Cannot compile comau_c5gopen package because COMAU eORL is not installed")
endif()


if(PREEMPTIVE_RT)
  message(-- " setting PREEMPTIVE_RT definition")
  add_definitions(-DPREEMPTIVE_RT)
endif()


if (EORL_AVAILABLE)
  add_executable(c5gopen_lpc_node src/c5gopen_lpc_node.cpp
                                  src/c5gopen_driver.cpp
                                  src/c5gopen_utilities.cpp
                                  src/realtime_utilities.cpp
                                  src/mqtt.cpp
                                  src/c5gopen_mqtt.cpp  
                                  src/dynamic_callback.cpp )

  add_library(eorl SHARED IMPORTED)
  set_property(TARGET eorl PROPERTY IMPORTED_LOCATION /opt/lib/libeORL.so)

  target_link_libraries(c5gopen_lpc_node  ${Boost_LIBRARIES}
                                          ${YAML_CPP_LIBRARIES}
                                          ${Mosquitto_LIBRARIES} 
                                          cnr_logger
                                          jsoncpp
                                          m 
                                          pthread 
                                          eorl )    
endif()
