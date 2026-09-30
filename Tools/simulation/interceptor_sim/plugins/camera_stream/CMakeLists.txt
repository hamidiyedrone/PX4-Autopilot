cmake_minimum_required(VERSION 3.16)
project(camera_stream CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE Release)
endif()

find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED IMPORTED_TARGET gstreamer-1.0 gstreamer-app-1.0)

add_library(CameraStream SHARED CameraStream.cc)
target_link_libraries(CameraStream PRIVATE
	gz-sim8::gz-sim8
	gz-plugin2::register
	gz-transport13::gz-transport13
	gz-msgs10::gz-msgs10
	PkgConfig::GST
)
