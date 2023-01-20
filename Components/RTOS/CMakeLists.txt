cmake_minimum_required(VERSION 3.10)

# Put directory name to COMPONENT_NAME variable
get_filename_component(COMPONENT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
# Set component name
project(${COMPONENT_NAME})


# Add you source file
file(GLOB_RECURSE SOURCE_LIB
    "FreeRTOS/*.c"
    "FreeRTOS/*.S"
)

# Add includes
include_directories(
    FreeRTOS/include
    FreeRTOS/portable/GCC/RISC-V
    Config
)

# Creating static library
add_library(${COMPONENT_NAME} STATIC ${SOURCE_LIB})

Depends_On(MLDR187_SPL)