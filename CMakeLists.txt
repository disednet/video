cmake_minimum_required(VERSION 3.10)
project(minimal_lpr)

find_package(OpenCV 4.3 REQUIRED)
find_package(Threads REQUIRED)
find_package(lprsdk REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(FFMPEG REQUIRED IMPORTED_TARGET
	libavdevice
	libavfilter
	libavformat
	libavcodec
	libswresample
	libswscale
	libavutil
)

add_executable(minimal_lpr minimal_lpr.cpp)
target_link_libraries(minimal_lpr PRIVATE LPRSDK::lprsdk ${OpenCV_LIBS} Threads::Threads)
target_include_directories(minimal_lpr PRIVATE ${OpenCV_INCLUDE_DIRS})
