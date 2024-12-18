cmake_minimum_required(VERSION 3.10)

project(rokae_imitation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(XCORESDK_PATH "/path/to/xCoreSDK-v0.4.1.b")
set(DH_PATH "/path/to/dh_gripper_ros")

add_library(Rokae STATIC IMPORTED)
add_library(xmatemodel_lib STATIC IMPORTED)

set_target_properties(xmatemodel_lib PROPERTIES
    IMPORTED_LOCATION ${XCORESDK_PATH}/lib/Linux/x86_64/libxMateModel.a)
set_target_properties(Rokae PROPERTIES
    IMPORTED_LOCATION ${XCORESDK_PATH}/lib/Linux/x86_64/libxCoreSDK.a
    INTERFACE_INCLUDE_DIRECTORIES ${XCORESDK_PATH}/include
    INTERFACE_LINK_LIBRARIES xmatemodel_lib
)

set(DH_INCLUDE ${DH_PATH}/dh_gripper_driver/include/dh_gripper_driver)
set(DH_SOURCE_FILES
    ${DH_INCLUDE}/src/dh_device.cpp
    ${DH_INCLUDE}/src/dh_ag95_can.cpp
    ${DH_INCLUDE}/src/dh_dh3_can.cpp
    ${DH_INCLUDE}/src/dh_lagacy_gripper.cpp
    ${DH_INCLUDE}/src/dh_modbus_gripper.cpp
    ${DH_INCLUDE}/src/dh_rgi.cpp
)

link_directories(/usr/lib/x86_64-linux-gnu)

find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)

include_directories(
    /usr/include
    ${PROJECT_SOURCE_DIR}/include
    ${DH_INCLUDE}/include
    ${XCORESDK_PATH}/include
    ${XCORESDK_PATH}/external
)

add_executable(gripper_control src/gripper_control.cpp ${DH_SOURCE_FILES})
add_executable(arm_control src/arm_control.cpp)

target_link_libraries(gripper_control zmq)
target_link_libraries(arm_control zmq Rokae Threads::Threads)
