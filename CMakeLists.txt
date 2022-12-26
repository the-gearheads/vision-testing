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

set(NtCore_INCLUDE_DIRS nt-vendor/ntcore/src/main/native/include nt-vendor/wpiutil/src/main/native/include)
include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable( vision main.cpp ApriltagDetect.cpp Config.cpp PhotonCompat.cpp )

# Kinda hacky but it works -enough- 

set(BUILD_SHARED_LIBS OFF)
set(BUILD_PYTHON_WRAPPER OFF)

add_subdirectory(apriltag EXCLUDE_FROM_ALL)

set(WITH_JAVA OFF)
set(WITH_TESTS OFF)
set(WPILIB_BINARY_DIR ${CMAKE_BINARY_DIR})
if(!MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-error=dangling-pointer=")
ENDIF()
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/nt-vendor/cmake/modules)
add_subdirectory(nt-vendor/wpiutil/ EXCLUDE_FROM_ALL)
add_subdirectory(nt-vendor/ntcore/ EXCLUDE_FROM_ALL)

target_link_libraries( vision apriltag ntcore ${OpenCV_LIBS} )
target_precompile_headers(vision PRIVATE pch.h)