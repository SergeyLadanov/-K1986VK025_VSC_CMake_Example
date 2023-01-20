cmake_minimum_required(VERSION 3.21)

# Put directory name to COMPONENT_NAME variable
get_filename_component(COMPONENT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
# Set component name
project(${COMPONENT_NAME})


# Add you source file
file(GLOB_RECURSE SOURCE_LIB
    "src/*.c"
)

# Add includes
include_directories(
    inc
)

# Creating static library
add_library(${COMPONENT_NAME} STATIC ${SOURCE_LIB})