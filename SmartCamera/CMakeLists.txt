cmake_minimum_required(VERSION 3.7)
project(SmartCamera)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES main.cpp MotionDetector.cpp MotionDetector.h Map.cpp Map.h)

add_executable(SmartCamera ${SOURCE_FILES})

find_package(OpenCV REQUIRED)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
