#
# Copyright (c) 2019, New York University and Max Planck Gesellschaft.
#
# License BSD-3 clause
#

#
# set up the project
#
cmake_minimum_required(VERSION 3.10.2)

project(robot_properties_iiwa)

# Using C++17
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

#
# Dependencies 
#
set(Python_ADDITIONAL_VERSIONS 3.6)
# depend on ament macros
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(mpi_cmake_modules REQUIRED)

# Export de dependencies.
ament_export_dependencies(
  ament_cmake 
  ament_cmake_python
  mpi_cmake_modules)

# prepare the final export
# ament_export_interfaces(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

#
# Install the python package.
#
ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR python/${PROJECT_NAME})

#
# Building documentation.
#
add_documentation()

#
# Export as an ament_package
#
ament_package()

