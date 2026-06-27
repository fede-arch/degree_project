#
# Development of this module has been funded by the Monterey Bay Aquarium
# Research Institute (MBARI) and the David and Lucile Packard Foundation
#

cmake_minimum_required(VERSION 3.5)

project(lrauv_description)

include(CTest)

# Model Generation
add_custom_command(
  OUTPUT modelsdf
  COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/scripts/description_generator.py
  ${CMAKE_CURRENT_SOURCE_DIR}/models/tethys/model.sdf.in
  ${CMAKE_CURRENT_BINARY_DIR}/models/tethys/model.sdf
)

add_custom_target(generate_model ALL
  DEPENDS modelsdf
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/models/tethys/model.sdf
  DESTINATION share/${PROJECT_NAME}/models/tethys
)
