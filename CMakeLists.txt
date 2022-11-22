cmake_minimum_required(VERSION 3.0)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

project( vision )
find_package( OpenCV REQUIRED )

set(CMAKE_CXX_STANDARD 20)

if(MSVC)
# Been struggling to get this working; cmake adds incompatible /rtc1
#	add_compile_options(/O2)
ELSE()
	add_compile_options(-Ofast)
ENDIF()

include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable( vision main.cpp ApriltagDetect.cpp )

# Kinda hacky but it works -enough- 

set(BUILD_SHARED_LIBS OFF)
set(BUILD_PYTHON_WRAPPER OFF)

add_subdirectory(apriltag EXCLUDE_FROM_ALL)
target_link_libraries( vision apriltag ${OpenCV_LIBS} )
target_precompile_headers(vision PRIVATE pch.h)