project(PARTICLE_FILTER)

cmake_minimum_required (VERSION 3.5)

set(CMAKE_BUILD_TYPE Release)

add_definitions(-std=c++11)
add_definitions()

set(CXX_FLAGS "-Wall -O2 -Os -march=native")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "-s")
## Compiler flags


set(sources src/particle_filter.cpp src/main.cpp src/utils.cpp)


if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 


add_executable(particle_filter ${sources})


target_link_libraries(particle_filter z ssl uv uWS)

