#
# Development of this module has been funded by the Monterey Bay Aquarium
# Research Institute (MBARI) and the David and Lucile Packard Foundation
#

cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(lrauv_gazebo_plugins)

include(CTest)

# Option to enable profiler
option(ENABLE_PROFILER "Enable Ignition Profiler" FALSE)

if(ENABLE_PROFILER)
  add_definitions("-DIGN_PROFILER_ENABLE=1")
else()
  add_definitions("-DIGN_PROFILER_ENABLE=0")
endif()

#============================================================================
# Find dependencies
find_package(gz-cmake3 REQUIRED)

find_package(gz-sim8 REQUIRED COMPONENTS gui)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})

find_package(gz-gui8 REQUIRED)
set(GZ_GUI_VER ${gz-gui8_VERSION_MAJOR})

find_package(gz-rendering8 REQUIRED)
set(GZ_RENDERING_VER ${gz-rendering8_VERSION_MAJOR})

find_package(gz-sensors8 REQUIRED)
set(GZ_SENSORS_VER ${gz-sensors8_VERSION_MAJOR})
set(GZ_SENSORS gz-sensors${GZ_SENSORS_VER}::gz-sensors${GZ_SENSORS_VER})

find_package(gz-msgs10 REQUIRED)
set(GZ_MSGS_VER ${gz-msgs10_VERSION_MAJOR})

find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

find_package(gz-utils2 REQUIRED)
set(GZ_UTILS_VER ${gz-utils2_VERSION_MAJOR})

find_package(gz-common5 REQUIRED COMPONENTS profiler)
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})

find_package (Eigen3 3.3 REQUIRED)

find_package(PCL 1.2 REQUIRED)

find_package(
  Python3
  COMPONENTS Interpreter)

# Build protobuf messages
#add_subdirectory(proto)
set(MSGS_PROTOS
     ${CMAKE_CURRENT_SOURCE_DIR}/proto/lrauv_gazebo_plugins/msgs/dvl_beam_state.proto
     ${CMAKE_CURRENT_SOURCE_DIR}/proto/lrauv_gazebo_plugins/msgs/dvl_kinematic_estimate.proto
     ${CMAKE_CURRENT_SOURCE_DIR}/proto/lrauv_gazebo_plugins/msgs/dvl_range_estimate.proto
     ${CMAKE_CURRENT_SOURCE_DIR}/proto/lrauv_gazebo_plugins/msgs/dvl_tracking_target.proto
     ${CMAKE_CURRENT_SOURCE_DIR}/proto/lrauv_gazebo_plugins/msgs/dvl_velocity_tracking.proto
     ${CMAKE_CURRENT_SOURCE_DIR}/proto/lrauv_gazebo_plugins/msgs/lrauv_acoustic_message.proto
     ${CMAKE_CURRENT_SOURCE_DIR}/proto/lrauv_gazebo_plugins/msgs/lrauv_command.proto
     ${CMAKE_CURRENT_SOURCE_DIR}/proto/lrauv_gazebo_plugins/msgs/lrauv_init.proto
     ${CMAKE_CURRENT_SOURCE_DIR}/proto/lrauv_gazebo_plugins/msgs/lrauv_internal_comms.proto
     ${CMAKE_CURRENT_SOURCE_DIR}/proto/lrauv_gazebo_plugins/msgs/lrauv_range_bearing_request.proto
     ${CMAKE_CURRENT_SOURCE_DIR}/proto/lrauv_gazebo_plugins/msgs/lrauv_range_bearing_response.proto
     ${CMAKE_CURRENT_SOURCE_DIR}/proto/lrauv_gazebo_plugins/msgs/lrauv_state.proto
)

# Call 'gz_msgs_generate_messages()' to process the .proto files
gz_msgs_generate_messages(
  # The cmake target to be generated for libraries/executables to link
  TARGET lrauv_gazebo_messages
  # The protobuf package to generate (Typically based on the path)
  PROTO_PACKAGE "lrauv_gazebo_plugins.msgs"
  # The path to the base directory of the proto files
  # All import paths should be relative to this (eg gz/custom_msgs/vector3d.proto)
  MSGS_PATH ${CMAKE_CURRENT_SOURCE_DIR}/proto
  # List of proto files to process
  MSGS_PROTOS ${MSGS_PROTOS}
  # Dependency on gz-msgs
  DEPENDENCIES gz-msgs${GZ_MSGS_VER}::gz-msgs${GZ_MSGS_VER}
)

#============================================================================
# Plugins

# add_lrauv_plugin (
#     <plugin_name>
#     [PROTO] msg1 msg2 ... [PCL] [GUI] [RENDERING]
#     [PRIVATE_LINK_LIBS] [INCLUDE_COMMS])
#
# Configure and install plugins.
#
# <plugin_name> Required. Name of the plugin library, which matches source file.
#
# [PROTO]: Optional. If included, plugin will be linked against custom protobuf
# messages that are listed.
#
# [PCL]: Optional. If included, plugin will be linked against PCL.
#
# [GUI]: Optional. Include for GUI plugins.
#
# [RENDERING]: Optional. Include to link against Ignition Rendering.
#
# [PRIVATE_LINK_LIBS]: Specify a list of libraries to be privately linked.
function(add_lrauv_plugin PLUGIN)
  set(options PCL GUI RENDERING SENSORS INCLUDE_COMMS)
  set(oneValueArgs)
  set(multiValueArgs PROTO PRIVATE_LINK_LIBS)

  cmake_parse_arguments(add_lrauv_plugin
    "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  set(PLUGIN_SOURCES src/${PLUGIN}.cc)
  set(INSTALL_LIB lib)

  # GUI plugin
  if (add_lrauv_plugin_GUI)
    QT5_WRAP_CPP(${PLUGIN}_headers_MOC src/${PLUGIN}.hh)
    QT5_ADD_RESOURCES(${PLUGIN}_RCC src/${PLUGIN}.qrc)

    set(PLUGIN_SOURCES ${PLUGIN_SOURCES}
      ${${PLUGIN}_headers_MOC}
      ${${PLUGIN}_RCC}
    )
    set(INSTALL_LIB lib/gui)
  endif()

  # All plugins
  add_library(${PLUGIN} SHARED ${PLUGIN_SOURCES})
  set_property(TARGET ${PLUGIN} PROPERTY CXX_STANDARD 17)
  target_link_libraries(${PLUGIN}
    PUBLIC
      gz-common${GZ_COMMON_VER}::profiler
      gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
      gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
    PRIVATE
      ${add_lrauv_plugin_PRIVATE_LINK_LIBS}
  )

  # Link against proto libraries
  if (add_lrauv_plugin_PROTO)
    target_link_libraries(${PLUGIN}
      PUBLIC ${add_lrauv_plugin_PROTO})

    target_include_directories(${PLUGIN}
      PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/${add_lrauv_plugin_PROTO}_genmsg)
  endif()

  # Include PCL
  if (add_lrauv_plugin_PCL)
    target_include_directories(${PLUGIN}
      PUBLIC ${PCL_INCLUDE_DIRS})
  endif()

  # GUI
  if (add_lrauv_plugin_GUI)
    target_link_libraries(${PLUGIN}
      PUBLIC
        TINYXML2::TINYXML2
      PRIVATE
        gz-gui${GZ_GUI_VER}::gz-gui${GZ_GUI_VER}
    )

    target_include_directories(${PLUGIN}
      PUBLIC ${PCL_INCLUDE_DIRS})
  endif()

  # Rendering
  if (add_lrauv_plugin_RENDERING)
    target_link_libraries(${PLUGIN}
      PRIVATE
        gz-rendering${GZ_RENDERING_VER}::gz-rendering${GZ_RENDERING_VER}
    )
  endif()

  install(
    TARGETS ${PLUGIN}
    DESTINATION ${INSTALL_LIB})
endfunction()

add_subdirectory(src/comms/)

add_lrauv_plugin(ControlPanelPlugin GUI
  PROTO ${PROJECT_NAME}-lrauv_gazebo_messages)
add_lrauv_plugin(DopplerVelocityLog
  RENDERING
  PROTO ${PROJECT_NAME}-lrauv_gazebo_messages)
target_link_libraries(DopplerVelocityLog PUBLIC ${GZ_SENSORS}-rendering)
add_lrauv_plugin(DopplerVelocityLogSystem RENDERING)
target_link_libraries(DopplerVelocityLogSystem PUBLIC
  DopplerVelocityLog ${GZ_SENSORS}-rendering)
add_lrauv_plugin(HydrodynamicsPlugin)
add_lrauv_plugin(RangeBearingPlugin
  PROTO
    ${PROJECT_NAME}-lrauv_gazebo_messages
  PRIVATE_LINK_LIBS
    acoustic_comms_support)
add_lrauv_plugin(ReferenceAxis GUI RENDERING)
add_lrauv_plugin(ScienceSensorsSystem
  PCL
  PRIVATE_LINK_LIBS
    ${GZ_SENSORS}
    ${PCL_LIBRARIES})
add_lrauv_plugin(SpawnPanelPlugin GUI
  PROTO ${PROJECT_NAME}-lrauv_gazebo_messages)
add_lrauv_plugin(TethysCommPlugin
  PROTO ${PROJECT_NAME}-lrauv_gazebo_messages)
add_lrauv_plugin(TimeAnalysisPlugin)
add_lrauv_plugin(WorldCommPlugin
  PROTO ${PROJECT_NAME}-lrauv_gazebo_messages)
add_lrauv_plugin(WorldConfigPlugin GUI)

#============================================================================
# Install public headers
install(
  DIRECTORY include/
    DESTINATION include
    FILES_MATCHING
    PATTERN "*.hh")

install(
  DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-lrauv_gazebo_messages_genmsg/lrauv_gazebo_plugins
    DESTINATION include
    FILES_MATCHING
    PATTERN "*.pb.h")

install(
  TARGETS ${PROJECT_NAME}-lrauv_gazebo_messages
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-lrauv_gazebo_messages.gz_desc
    DESTINATION proto)

#============================================================================
# Examples
foreach(EXAMPLE
  example_buoyancy
  example_controller
  example_comms_client
  example_comms_echo
  example_dvl_velocity
  example_elevator
  example_mass_shifter
  example_rudder
  example_spawn
  example_thruster
  keyboard_teleop
  multi_lrauv_race)

  set(EXAMPLE_EXEC LRAUV_${EXAMPLE})

  add_executable(${EXAMPLE_EXEC} example/${EXAMPLE}.cc)
  set_property(TARGET ${EXAMPLE_EXEC} PROPERTY CXX_STANDARD 17)
  target_include_directories(${EXAMPLE_EXEC}
    PUBLIC
      include
      ${CMAKE_CURRENT_BINARY_DIR}/${add_lrauv_plugin_PROTO}_genmsg)
  target_link_libraries(${EXAMPLE_EXEC} PRIVATE
    gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
    acoustic_comms_support
    ${PROJECT_NAME}-lrauv_gazebo_messages)

  install(
    TARGETS ${EXAMPLE_EXEC}
    DESTINATION bin)

endforeach()

#============================================================================
# Exports
install(
  EXPORT ${PROJECT_NAME}
  FILE ${PROJECT_NAME}-targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake)

include(CMakePackageConfigHelpers)
configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/Config.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  INSTALL_DESTINATION "share/${PROJECT_NAME}/cmake"
  NO_SET_AND_CHECK_MACRO
  NO_CHECK_REQUIRED_COMPONENTS_MACRO)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  DESTINATION share/${PROJECT_NAME}/cmake)

#============================================================================
# Hooks
configure_file(
  "hooks/hook.dsv.in"
  "${CMAKE_CURRENT_BINARY_DIR}/hooks/hook.dsv" @ONLY
)
configure_file(
  "hooks/hook.sh.in"
  "${CMAKE_CURRENT_BINARY_DIR}/hooks/hook.sh" @ONLY
)

#============================================================================
# World generation

if (${Python3_FOUND})
  execute_process(
      COMMAND ${Python3_EXECUTABLE} -c "import gz"
      RESULT_VARIABLE EXIT_CODE
      OUTPUT_QUIET
  )

  if (${EXIT_CODE} EQUAL 0)
    foreach (WORLD_NAME "portuguese_ledge" "tethys_at_portuguese_ledge" "race_at_portuguese_ledge")
      add_custom_command(
        OUTPUT ${WORLD_NAME}_gen_cmd
        COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/worlds/empy_expander.py
        ${CMAKE_CURRENT_SOURCE_DIR}/worlds/${WORLD_NAME}.sdf.em
        ${CMAKE_CURRENT_BINARY_DIR}/worlds/${WORLD_NAME}.sdf
      )

      add_custom_target(${WORLD_NAME}_gen_target ALL
        DEPENDS ${WORLD_NAME}_gen_cmd
      )

      install(FILES
        ${CMAKE_CURRENT_BINARY_DIR}/worlds/${WORLD_NAME}.sdf
        DESTINATION share/${PROJECT_NAME}/worlds)
    endforeach()
  else()
    message(
      WARNING "Could not find the Gazebo Python bindings, some worlds will not be generated.\
      If you installed gazebo from source, make sure you set the PYTHON_PATH like so\
      \n\t export PYTHONPATH=$PYTHONPATH:<path to ws>/install/lib/python")
  endif()
else()
  message(WARNING "Python not found. Some worlds may not be generated.")
endif()
#============================================================================
# Resources
install(
  DIRECTORY
    data
    models
    worlds
    ${CMAKE_CURRENT_BINARY_DIR}/hooks
  DESTINATION share/${PROJECT_NAME})
