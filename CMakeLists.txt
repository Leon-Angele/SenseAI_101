cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME SenseAI_101)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Option to enable CMSIS-DSP optimizations (requires CMSIS-DSP sources present)
option(USE_CMSIS_DSP "Enable CMSIS-DSP optimizations" OFF)
if(USE_CMSIS_DSP)
    message(STATUS "CMSIS-DSP support enabled")
    # Expect CMSIS-DSP to be located under Drivers/CMSIS/DSP
    set(CMSIS_DSP_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/DSP/Include)
    set(CMSIS_DSP_SRC_DIR ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/DSP/Source)
    if(EXISTS ${CMSIS_DSP_INCLUDE_DIR} AND EXISTS ${CMSIS_DSP_SRC_DIR})
        target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${CMSIS_DSP_INCLUDE_DIR})
        # Add all DSP source files (C) under the source tree
        file(GLOB_RECURSE CMSIS_DSP_SOURCES ${CMSIS_DSP_SRC_DIR}/*.c)
        if(CMSIS_DSP_SOURCES)
            target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${CMSIS_DSP_SOURCES})
        endif()
        target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE USE_CMSIS_DSP ARM_MATH_CM4)
    else()
        message(FATAL_ERROR "USE_CMSIS_DSP requested but Drivers/CMSIS/DSP not found. Please add CMSIS-DSP sources to Drivers/CMSIS/DSP or disable USE_CMSIS_DSP.")
    endif()
endif()

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    Core/Src/kinematics.c
    Core/Src/servo_protocol.c
    Core/Src/servo_mapping.c
    Core/Src/trajectory.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Inc
    so101_IK/src
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
