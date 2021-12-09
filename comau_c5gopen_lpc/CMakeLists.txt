cmake_minimum_required(VERSION 2.8.3)
project(comau_c5gopen_lpc)

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -pg -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 ")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--copy-dt-needed-entries")

find_package( Boost REQUIRED COMPONENTS system filesystem )

include_directories(include
                    ${Boost_INCLUDE_DIRS}
                    /opt/inc/
                    )

link_directories(${Boost_LIBRARY_DIR})                


if(EXISTS "/opt/inc/eORL.h")
  MESSAGE("***** eORL ENABLED *****")
  option(EORL_ENABLED "eORL ENABLED" ON)
  add_definitions(-DEORL_AVAILABLE)  
 else()
  MESSAGE("***** eORL NOT ENABLED *****")
  option(EORL_ENABLED "eORL ENABLED" OFF)
  message(WARNING "Cannot compile comau_c5gopen package because COMAU eORL is not installed")
endif()

add_executable(c5gopen_lpc_node src/c5gopen_lpc_node.cpp)

if (EORL_ENABLE)
  add_library(eorl SHARED IMPORTED)
  set_property(TARGET eorl PROPERTY IMPORTED_LOCATION /opt/lib/libeORL.so)

  target_link_libraries(c5gopen_lpc_node ${Boost_LIBRARIES} 
                                          cnr_logger
                                          m 
                                          pthread 
                                          eorl )    
else()
  target_link_libraries(c5gopen_lpc_node  ${Boost_LIBRARIES} 
                                          m 
                                          pthread 
                                          cnr_logger )    
endif()




