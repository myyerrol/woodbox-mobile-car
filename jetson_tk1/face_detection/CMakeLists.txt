cmake_minimum_required(VERSION 2.8.3)
project(face_detection)

find_package(OpenCV)

add_executable(
    face_detection
    face_detection.cpp
)

target_link_libraries(
    face_detection
    ${OpenCV_LIBRARIES}
)
