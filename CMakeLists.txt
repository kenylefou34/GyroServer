cmake_minimum_required(VERSION 3.5)

project(GyroClient VERSION 0.1 LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(SRC_DIR src)
set(APPS_DIR apps)
set(UI_DIR ui)
set(UDP_DIR udp)
set(TCP_DIR tcp)

# Frame from udp/tcp executable

set(SOURCES
    ${SRC_DIR}/${UDP_DIR}/server.cpp
    ${SRC_DIR}/${TCP_DIR}/server.cpp
)

add_executable(stream_frames
  ${SRC_DIR}/${APPS_DIR}/stream_frames.cpp
  ${SOURCES})

# Optionally, include directories with header files
# target_include_directories(stream_frames
#     PRIVATE
#     ${SRC_DIR}/tcp
#     ${SRC_DIR}/udp
#     ${SRC_DIR}/types
# )

find_package(OpenCV REQUIRED)

target_link_libraries(stream_frames
  PRIVATE
  ${OpenCV_LIBS}
)

# H264 executable
find_package(PkgConfig REQUIRED)
pkg_check_modules(FFMPEG REQUIRED libavcodec libavformat libswscale)

add_executable(stream_media ${SRC_DIR}/${APPS_DIR}/stream_media.cpp)

target_include_directories(stream_media
  PRIVATE
  ${OpenCV_INCLUDE_DIRS}
  ${FFMPEG_INCLUDE_DIRS})

target_link_libraries(stream_media
  PRIVATE
  ${OpenCV_LIBS}
  ${FFMPEG_LIBRARIES}
  avcodec
  avformat
  avutil
  swscale
)
