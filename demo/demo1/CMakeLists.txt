cmake_minimum_required(VERSION 3.12)

project(Demo1)

# aris use c++ 17 standard
set(CMAKE_CXX_STANDARD 17)

# find aris
# if(UNIX)
# 	add_definitions(-DUNIX)
#     find_package(aris REQUIRED PATHS ${PROJECT_SOURCE_DIR}/lib/aris NO_DEFAULT_PATH)
#     find_package(aris REQUIRED PATHS /usr/aris)
# endif(UNIX)
# IF(WIN32)
#         find_package(aris REQUIRED PATHS C:/aris/aris-1.5.0)
# endif(WIN32)
# include_directories(${aris_INCLUDE_DIRS})

#
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(testmotor src/main.cpp src/robot.cpp  src/plan.cpp)

if(UNIX)
    target_link_libraries(testmotor ${aris_LIBRARIES} atomic)
endif(UNIX)
IF(WIN32)
    target_link_libraries(testmotor ${aris_LIBRARIES})
endif(WIN32)

