#Hey there! You need to install OpenCV first!

cmake_minimum_required(VERSION 3.2)
project(Aura CXX)
find_package(OpenCV REQUIRED)
find_package (Threads)
set (CMAKE_CXX_STANDARD 11)
add_executable(videostab src/videostab/videostab.cpp)
add_executable(rawanalyze src/rawanalyze/rawanalyze.cpp)
add_executable(sidebyside src/sidebyside/sidebyside.cpp)
target_link_libraries(videostab ${OpenCV_LIBS} ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(sidebyside ${OpenCV_LIBS})

#Enable C++11
#target_compile_features(videostab PRIVATE cxx_range_for)
#target_compile_features(rawanalyze PRIVATE cxx_range_for)
#target_compile_features(sidebyside PRIVATE cxx_range_for)

#Add special windows DLLS
#if(MSVC)
#add_custom_command(TARGET rawanalyze POST_BUILD
# COMMAND ${CMAKE_COMMAND} -E copy_directory
#"${PROJECT_SOURCE_DIR}/lib"
#        $<TARGET_FILE_DIR:rawanalyze>)
#endif()
