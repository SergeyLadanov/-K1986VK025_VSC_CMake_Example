cmake_minimum_required(VERSION 3.21)

# Get current path
set(path ${CMAKE_CURRENT_SOURCE_DIR})
# Put directory name to COMPONENT_NAME variable
cmake_path(GET path FILENAME COMPONENT_NAME)
# Set component name
project(${COMPONENT_NAME})


# Add source files
file(GLOB_RECURSE SOURCE_LIB 
    "src/*.c"
    "src/*.cpp"
    "MLDR187_SPL/*.c"
)


# Creating static library
add_library(${COMPONENT_NAME} STATIC ${SOURCE_LIB})

all_dependencies()