
cmake_minimum_required (VERSION 3.13)
project(SensorServer)
set (CMAKE_CXX_STANDARD 11)
# Add source to this project's executable.
add_executable (SensorServer)
target_include_directories(SensorServer PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../inc)
target_sources(SensorServer PRIVATE
				${CMAKE_CURRENT_SOURCE_DIR}/src/SensorServer.cpp
				${CMAKE_CURRENT_SOURCE_DIR}/../inc/Definitions.h) 
				
if(CMAKE_HOST_WIN32)
  target_link_libraries(SensorServer ws2_32.lib)
endif()



