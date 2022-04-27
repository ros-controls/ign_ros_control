cmake_minimum_required(VERSION 3.5)
project(ign_ros_control)

# Default to C11
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 11)
endif()
# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(catkin REQUIRED COMPONENTS
    roscpp
    controller_manager
    transmission_interface
    hardware_interface
    pluginlib
    urdf
    )

if("$ENV{IGNITION_VERSION}" STREQUAL "citadel")
  find_package(ignition-gazebo3 REQUIRED)
  set(IGN_GAZEBO_VER ${ignition-gazebo3_VERSION_MAJOR})
  message(STATUS "Compiling against Ignition Citadel")

elseif("$ENV{IGNITION_VERSION}" STREQUAL "dome")
  find_package(ignition-gazebo4 REQUIRED)
  set(IGN_GAZEBO_VER ${ignition-gazebo4_VERSION_MAJOR})
  message(STATUS "Compiling against Ignition Dome")

elseif("$ENV{IGNITION_VERSION}" STREQUAL "edifice")
  find_package(ignition-gazebo5 REQUIRED)
  set(IGN_GAZEBO_VER ${ignition-gazebo5_VERSION_MAJOR})
  message(STATUS "Compiling against Ignition Edifice")

elseif("$ENV{IGNITION_VERSION}" STREQUAL "fortress")
  find_package(ignition-gazebo6 REQUIRED)
  set(IGN_GAZEBO_VER ${ignition-gazebo6_VERSION_MAJOR})
  message(STATUS "Compiling against Ignition Fortress")

else()
  find_package(ignition-gazebo4 REQUIRED)
  set(IGN_GAZEBO_VER ${ignition-gazebo4_VERSION_MAJOR})
  message(STATUS "Compiling against Ignition Dome")
endif()

find_package(ignition-plugin1 REQUIRED)
set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})

catkin_package(
  LIBRARIES ign_ros_control_system ign_ros_control_plugin
  INCLUDE_DIRS include)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(ign_ros_control_system SHARED
  src/ign_system.cpp
)
target_link_libraries(ign_ros_control_system
  ignition-gazebo${IGN_GAZEBO_VER}::core
  ignition-plugin${IGN_PLUGIN_VER}::register
  ${catkin_LIBRARIES}
)

add_library(ign_ros_control_plugin SHARED
  src/ign_ros_control_plugin.cpp
)
target_link_libraries(ign_ros_control_plugin
  ign_ros_control_system
  ignition-gazebo${IGN_GAZEBO_VER}::core
  ${catkin_LIBRARIES}
)

# Install directories
install(TARGETS ign_ros_control_system ign_ros_control_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(FILES ign_ros_control_plugins.xml
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
