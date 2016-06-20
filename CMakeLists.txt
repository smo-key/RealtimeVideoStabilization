#Hey there! You need to install OpenCV and CUDA first!

cmake_minimum_required(VERSION 3.2)
project(Aura)
find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)
include_directories(${CUDA_SOURCE_DIR}/include)
#include(${CUDA_USE_FILE})

cuda_add_executable(videostab src/videostab/videostab.cpp)
target_link_libraries(videostab ${OpenCV_LIBS} ${CUDA_LIBRARIES})

cuda_add_executable(sidebyside src/sidebyside/sidebyside.cpp)
target_link_libraries(sidebyside ${OpenCV_LIBS} ${CUDA_LIBS})

add_executable(rawanalyze src/rawanalyze/rawanalyze.cpp)

#Add special windows DLLS
#if(MSVC)
#add_custom_command(TARGET rawanalyze POST_BUILD
# COMMAND ${CMAKE_COMMAND} -E copy_directory
#"${PROJECT_SOURCE_DIR}/lib"
#        $<TARGET_FILE_DIR:rawanalyze>)
#endif()