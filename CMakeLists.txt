cmake_minimum_required(VERSION 3.10)

project(cartpole)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS "-O3 -pthread")

set(MUJOCO_DIR /home/wcompton/Repos/mujoco-2.3.5-linux-x86_64/mujoco-2.3.5)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(glfw3 REQUIRED)

include_directories(${MUJOCO_DIR}/include)
include_directories(${MUJOCO_DIR}/include/mujoco)
find_library(MUJOCO_LIB mujoco REQUIRED HINTS ${MUJOCO_DIR}/lib)

add_library(mujoco SHARED IMPORTED GLOBAL)
set_property(TARGET mujoco PROPERTY IMPORTED_LOCATION ${CMAKE_INCLUDE_DIR}/lib/libmujoco.so)

include_directories(include)

add_library(socket SHARED src/socket.cpp include/socket.h)
add_library(utilities SHARED src/utilities.cpp include/utilities.h)
add_library(mujoco_interface SHARED src/mujoco_interface.cpp include/mujoco_interface.h)
add_library(lqr SHARED src/LQR.cpp include/LQR.h)

add_executable(cartpole_sim src/sim.cpp)
target_link_libraries(cartpole_sim PUBLIC socket)
target_link_libraries(cartpole_sim PUBLIC mujoco_interface)
target_link_libraries(cartpole_sim PUBLIC utilities)
target_link_libraries(cartpole_sim PUBLIC Eigen3::Eigen)
target_link_libraries(cartpole_sim PUBLIC glfw)
target_link_libraries(cartpole_sim PUBLIC yaml-cpp)
target_link_libraries(cartpole_sim PUBLIC ${MUJOCO_LIB})
target_link_directories(cartpole_sim PUBLIC ${MUJOCO_DIR})

add_executable(cartpole_controller src/controller.cpp)
target_link_libraries(cartpole_controller yaml-cpp)
target_link_libraries(cartpole_controller socket)
target_link_libraries(cartpole_controller lqr)


add_executable(test_socket_client test/test_socket_client.cpp)
target_link_libraries(test_socket_client socket)

add_executable(test_socket_server test/test_socket_server.cpp)
target_link_libraries(test_socket_server socket)