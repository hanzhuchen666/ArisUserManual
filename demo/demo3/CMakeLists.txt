cmake_minimum_required(VERSION 3.12)

project(Demo3)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(TorqueControl src/main.cpp src/robot.cpp  src/plan.cpp)

if(UNIX)
    target_link_libraries(TorqueControl ${aris_LIBRARIES} atomic)
endif(UNIX)
IF(WIN32)
    target_link_libraries(TorqueControl ${aris_LIBRARIES})
endif(WIN32)

