cmake_minimum_required(VERSION 3.7)
project(Task3_1)

set(CMAKE_CXX_STANDARD 14)

set(SOURCE_FILES main.cpp)
add_executable(Task3_1 ${SOURCE_FILES})
find_package(OpenCV REQUIRED)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
