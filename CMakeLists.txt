cmake_minimum_required(VERSION 3.0)
project( vision )
find_package( OpenCV REQUIRED )
add_compile_options(-Ofast)
include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable( vision main.cpp )

# Kinda hacky but it works -enough- 

set(BUILD_SHARED_LIBS OFF)
set(BUILD_PYTHON_WRAPPER OFF)

add_subdirectory(apriltag EXCLUDE_FROM_ALL)
target_link_libraries( vision apriltag ${OpenCV_LIBS} )