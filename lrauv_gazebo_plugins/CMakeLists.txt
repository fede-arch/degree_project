cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(lrauv_gazebo_plugins)

find_package(gz-cmake3 REQUIRED)
find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})

find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

find_package(gz-common5 REQUIRED COMPONENTS profiler)
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})

find_package(Eigen3 3.3 REQUIRED)

add_library(HydrodynamicsPlugin SHARED src/HydrodynamicsPlugin.cc)
set_property(TARGET HydrodynamicsPlugin PROPERTY CXX_STANDARD 17)
target_link_libraries(HydrodynamicsPlugin
  PUBLIC
    gz-common${GZ_COMMON_VER}::profiler
    gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
    gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
)
install(TARGETS HydrodynamicsPlugin DESTINATION lib)
