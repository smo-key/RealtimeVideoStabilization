#Hey there! You need to install OpenCV first!

cmake_minimum_required(VERSION 3.2)
project(Aura)
find_package(OpenCV REQUIRED)
add_executable(videostab src/videostab/videostab.cpp)
add_executable(rawanalyze src/rawanalyze/rawanalyze.cpp)
target_link_libraries(videostab ${OpenCV_LIBS})