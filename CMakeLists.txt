cmake_minimum_required(VERSION 3.12)

project(sixaxisrobot)

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

# find kaanh
# if(UNIX)
#     add_definitions(-DUNIX)
#     find_package(kaanh REQUIRED PATHS ${PROJECT_SOURCE_DIR}/lib/kaanh NO_DEFAULT_PATH)
#     find_package(kaanh REQUIRED PATHS /usr/kaanh)
# endif(UNIX)
# IF(WIN32)
#     find_package(kaanh REQUIRED PATHS C:/kaanh/kaanh-1.0)
# endif(WIN32)
# include_directories(${kaanh_INCLUDE_DIRS})

message(${aris_DIR})
message(${kaanh_DIR})
message(${aris_INCLUDE_DIRS})


# option(USE_OPTITRACK "Use the function of optitrack devices" ON)
# option(USE_IMU "Use the function of Imu devices" OFF)
# configure_file(Config.h.in ${PROJECT_BINARY_DIR}/include/Config.h)
# include_directories(${PROJECT_SOURCE_DIR}/include)
# include_directories(${PROJECT_BINARY_DIR}/include)

# if(USE_OPTITRACK)
#     link_directories(${PROJECT_SOURCE_DIR}/lib)
#     add_executable(SampleClient ${PROJECT_SOURCE_DIR}/src/SampleClient.cpp)
#     target_link_libraries(SampleClient ${PROJECT_SOURCE_DIR}/lib/libNatNet.so )
# endif(USE_OPTITRACK)


# add_executable(sixaxisrobot src/tools.cpp src/main.cpp src/robot.cpp  src/imu.cpp)


file(COPY src/kaanh.xml DESTINATION ./)

if(UNIX)
    target_link_libraries(sixaxisrobot ${kaanh_LIBRARIES} ${aris_LIBRARIES}  atomic)
endif(UNIX)
IF(WIN32)
    target_link_libraries(sixaxisrobot ${kaanh_LIBRARIES} ${aris_LIBRARIES})
endif(WIN32)

