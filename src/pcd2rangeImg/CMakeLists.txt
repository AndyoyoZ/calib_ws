cmake_minimum_required(VERSION 2.8.3)

add_definitions(-std=c++11)

project(pcd2rangeImg)

find_package(PCL REQUIRED)


link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

file(GLOB native_srcs "src/*.cpp")

add_executable(${PROJECT_NAME} ${native_srcs})


target_link_libraries(${PROJECT_NAME}
   ${PCL_LIBRARIES}
 )


