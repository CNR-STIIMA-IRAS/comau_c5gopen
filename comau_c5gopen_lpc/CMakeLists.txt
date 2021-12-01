cmake_minimum_required(VERSION 2.8.3)
#project(comau_c5gopen_lpc)

#set(CMAKE_CXX_FLAGS "-std=c++11")
#set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -pg -Wall")
#set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 ")

#set(dir ${CMAKE_CURRENT_SOURCE_DIR}/build/)
#set(EXECUTABLE_OUTPUT_PATH ${dir} CACHE PATH "Build directory" FORCE)
#set(LIBRARY_OUTPUT_PATH ${dir} CACHE PATH "Build directory" FORCE)
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${dir})
#set(CMAKE_BUILD_FILES_DIRECTORY ${dir})
#set(CMAKE_BUILD_DIRECTORY ${dir})
#set(CMAKE_BINARY_DIR  ${dir})
#SET(EXECUTABLE_OUTPUT_PATH ${dir})
#SET(LIBRARY_OUTPUT_PATH ${dir}lib)
#SET(CMAKE_CACHEFILE_DIR ${dir})

#find_package( Boost REQUIRED COMPONENTS system filesystem )

#include_directories(include
#                    ${BOOST_INCLUDE_DIRS}  
#                    )

#include_directories( /opt/inc/ )                


#if(EXISTS "/opt/inc/eORL.h")
#  add_executable(c5gopen_lpc_node src/c5gopen_lpc_node.cpp)

#  add_library(eorl SHARED IMPORTED)
#  set_property(TARGET eorl PROPERTY IMPORTED_LOCATION /opt/lib/libeORL.so)
  
#  target_link_libraries(c5gopen_lpc_node eorl m pthread)    
                                             
# else()
#  message(WARNING "Cannot compile comau_c5gopen package because COMAU eORL is not installed")
#endif()


