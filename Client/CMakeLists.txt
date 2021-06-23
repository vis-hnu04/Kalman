cmake_minimum_required (VERSION 3.13)
project(ObjectFusion)
set (CMAKE_CXX_STANDARD 11)
# Add source to this project's executable.
add_executable (ObjectFusion)
target_include_directories(ObjectFusion PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/inc ${CMAKE_CURRENT_SOURCE_DIR}/../inc)
target_sources(ObjectFusion PRIVATE
			${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
			${CMAKE_CURRENT_SOURCE_DIR}/src/JSONFileLogger.cpp
			${CMAKE_CURRENT_SOURCE_DIR}/src/KalmanTracker.cpp
			${CMAKE_CURRENT_SOURCE_DIR}/../inc/Definitions.h
			${CMAKE_CURRENT_SOURCE_DIR}/inc/ISensorInterface.h
			${CMAKE_CURRENT_SOURCE_DIR}/inc/IFusionInterface.h
			${CMAKE_CURRENT_SOURCE_DIR}/inc/ISensorImplementation.h
			${CMAKE_CURRENT_SOURCE_DIR}/inc/KalmanTracker.h
			${CMAKE_CURRENT_SOURCE_DIR}/inc/HungarianAlgorithm.h
			${CMAKE_CURRENT_SOURCE_DIR}/src/HungarianAlgorithm.cpp
			${CMAKE_CURRENT_SOURCE_DIR}/inc/matrix.h
            ${CMAKE_CURRENT_SOURCE_DIR}/src/matrix.cpp)                       
	        	
			
if(CMAKE_HOST_WIN32)
  target_link_libraries(ObjectFusion ws2_32.lib)
endif()
