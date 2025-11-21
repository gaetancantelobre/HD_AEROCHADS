cmake_minimum_required(VERSION 3.5)

# 1. Define the project name first
project(control_maquette)

# 2. Include the boilerplate for ESP-IDF
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# The project.cmake file automatically looks for components in the 'main' directory. 
# DO NOT include component-specific commands like target_link_libraries or register_component here.