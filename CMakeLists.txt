#Hey there! You need to install OpenCV first!

cmake_minimum_required(VERSION 3.2)
project(Aura)
find_package(OpenCV REQUIRED)
add_executable(videostab src/videostab/videostab.cpp)
add_executable(rawanalyze src/rawanalyze/rawanalyze.cpp)
add_executable(sidebyside src/sidebyside/sidebyside.cpp)
target_link_libraries(videostab ${OpenCV_LIBS})
target_link_libraries(sidebyside ${OpenCV_LIBS})

#Add special windows DLLS
#if(MSVC)
#add_custom_command(TARGET rawanalyze POST_BUILD
# COMMAND ${CMAKE_COMMAND} -E copy_directory
#"${PROJECT_SOURCE_DIR}/lib"
#        $<TARGET_FILE_DIR:rawanalyze>)
#endif()