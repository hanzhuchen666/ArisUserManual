cmake_minimum_required(VERSION 3.12)

project(ARISUSERMANUAL)

# aris use c++ 17 standard
set(CMAKE_CXX_STANDARD 17)

# find aris
if(UNIX)
    add_definitions(-DUNIX)
    find_package(aris REQUIRED PATHS ${PROJECT_SOURCE_DIR}/lib/aris NO_DEFAULT_PATH)
    find_package(aris REQUIRED PATHS /usr/aris)
endif(UNIX)
IF(WIN32)
    find_package(aris REQUIRED PATHS C:/aris/aris-1.5.0)
endif(WIN32)
include_directories(${aris_INCLUDE_DIRS})


message(${aris_DIR})
message(${aris_INCLUDE_DIRS})

add_subdirectory(${PROJECT_SOURCE_DIR}/demo/demo1)
add_subdirectory(${PROJECT_SOURCE_DIR}/demo/demo2)
add_subdirectory(${PROJECT_SOURCE_DIR}/demo/demo3)



