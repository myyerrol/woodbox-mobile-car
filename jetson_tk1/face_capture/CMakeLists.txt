cmake_minimum_required(VERSION 2.8.3)
project(face_capture)

find_package(OpenCV)

add_executable(
    face_capture
    face_capture.cpp
)

target_link_libraries(
    face_capture
    ${OpenCV_LIBRARIES}
)
