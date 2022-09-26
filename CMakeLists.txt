cmake_minimum_required(VERSION 3.21)


set(APP_NAME "Milandr_Template")

# Add you source file
file(GLOB_RECURSE SOURCES
    "src/startup_MLDR187.S"
    "src/*.c"
    "src/*.cpp"
)


set(INCLUDE_LIST

)


macro(__Get_DirectorytList result curdir)
  file(GLOB children RELATIVE ${curdir} ${curdir}/*)
  set(dirlist "")
  foreach(child ${children})
    if(IS_DIRECTORY ${curdir}/${child})
      list(APPEND dirlist ${child})
    endif()
  endforeach()
  set(${result} ${dirlist})
endmacro()


macro(__Get_IncludeDirectories result curdir)
  file(GLOB_RECURSE children ${curdir}/*.h ${curdir}/*.hpp)
  message("Adding include directories for \"${PROJECT_NAME}\"...")
  set(dirlist "")
  set(curItem "")
  foreach(child ${children})

    # Get current path
    set(path ${child})
    cmake_path(GET path PARENT_PATH newResult)

    if (NOT curItem STREQUAL newResult)
        message("Add \"${newResult}\" to icnlude directory list")
        list(APPEND dirlist ${newResult})
        set(curItem ${newResult})
    endif()
    
  endforeach()
  set(${result} ${dirlist})
endmacro()


set(COMPONENTS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/Components)

__Get_DirectorytList(COMPONENT_LIST ${COMPONENTS_DIRECTORY})

message("Project components: \"${COMPONENT_LIST}\"")


function(Depends_On name)
    message("Add dependency \"${name}\" for \"${PROJECT_NAME}\"")
    set(dirs "")
    __Get_IncludeDirectories(dirs ${COMPONENTS_DIRECTORY}/${name})
    target_include_directories(${PROJECT_NAME} PRIVATE ${dirs})
    target_link_libraries(${PROJECT_NAME} ${name})
endfunction()


function(Depends_All)
    if (COMPONENT_LIST)
        foreach(item ${COMPONENT_LIST})
            Depends_On(${item})
        endforeach()
    endif()
endfunction()


function(__Register_Component name)
    add_subdirectory(Components/${name})
    get_property(inc_dirs DIRECTORY ${COMPONENTS_DIRECTORY}/${name} PROPERTY INCLUDE_DIRECTORIES)
    target_include_directories(${PROJECT_NAME}.elf PRIVATE ${inc_dirs}) 
    target_link_libraries(${PROJECT_NAME}.elf ${name})
endfunction()


function(__Add_Components components_list_in)
    set( _components_list ${components_list_in} ${ARGN} )
    foreach(item ${_components_list})
        __Register_Component(${item})
    endforeach()
endfunction()


set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)

 
set(CMAKE_C_COMPILER riscv-none-embed-gcc)
set(CMAKE_CXX_COMPILER riscv-none-embed-g++)
set(CMAKE_ASM_COMPILER riscv-none-embed-gcc)
set(CMAKE_AR riscv-none-embed-gcc-ar)
set(CMAKE_OBJCOPY riscv-none-embed-objcopy)
set(CMAKE_OBJDUMP riscv-none-embed-objdump)
set(SIZE riscv-none-embed-size)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
 
# Add you project name
project(${APP_NAME} C CXX ASM)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)
 
# Add carnel name
set(MCPU cortex-m4)
 
# Uncomment for hardware floating point
#add_compile_definitions(ARM_MATH_CM4;ARM_MATH_MATRIX_CHECK;ARM_MATH_ROUNDING)
#add_compile_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)
#add_link_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)
 
# Uncomment for software floating point
#add_compile_options(-mfloat-abi=soft)
 
# add_compile_options(-mcpu=${MCPU} -mthumb -mthumb-interwork)
add_compile_options(-ffunction-sections -fdata-sections -fno-common -fmessage-length=0)
 
# Uncomment to mitigate c++17 absolute addresses warnings
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-register")
 
if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    message(STATUS "Maximum optimization for speed")
    add_compile_options(-Ofast)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "RelWithDebInfo")
    message(STATUS "Maximum optimization for speed, debug info included")
    add_compile_options(-Ofast -g)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "MinSizeRel")
    message(STATUS "Maximum optimization for size")
    add_compile_options(-Os)
else ()
    message(STATUS "Minimal optimization, debug info included")
    add_compile_options(-Og -g)
endif ()
 

 
# Add C defines
add_definitions(-DHXTAL_VALUE=25000000 -D__SYSTEM_CLOCK_108M_PLL_HXTAL=108000000 -DRISCV_ALL_FFT_TABLES)
 
 
# Add lincer file
set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/src/link_Flash.ld)
 
# this options for C++
add_link_options(-march=rv32imc -mabi=ilp32 -mcmodel=medany -msmall-data-limit=8 -mno-save-restore -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-move-loop-invariants -Wall -Wextra -g3)
add_link_options(-T ${LINKER_SCRIPT})
add_link_options(-nostartfiles -Xlinker --gc-sections)
add_link_options(-Wl,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map)
add_link_options(--specs=nano.specs)




 
add_executable(${PROJECT_NAME}.elf ${SOURCES} ${LINKER_SCRIPT})

# Директории с *.h файлами
target_include_directories(${PROJECT_NAME}.elf PRIVATE
    ${INCLUDE_LIST}
)

if (COMPONENT_LIST)
    __Add_Components(${COMPONENT_LIST})
endif()





 
set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)
 
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMAND ${SIZE} $<TARGET_FILE:${PROJECT_NAME}.elf>
        COMMENT "Generated Hex file: ${HEX_FILE}
Generated Bin file: ${BIN_FILE}")
