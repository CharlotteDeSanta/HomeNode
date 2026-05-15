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
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME HomeNode)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Node role selection (compile-time).
# Values: kitchen / living / bedroom
set(HOME_NODE_ROLE "kitchen" CACHE STRING "Node role for HomeNode (kitchen|living|bedroom)")
set_property(CACHE HOME_NODE_ROLE PROPERTY STRINGS kitchen living bedroom)
string(TOLOWER "${HOME_NODE_ROLE}" HOME_NODE_ROLE_LOWER)

if(HOME_NODE_ROLE_LOWER STREQUAL "kitchen")
    set(HOME_NODE_ID_VALUE 1)
elseif(HOME_NODE_ROLE_LOWER STREQUAL "living")
    set(HOME_NODE_ID_VALUE 2)
elseif(HOME_NODE_ROLE_LOWER STREQUAL "bedroom")
    set(HOME_NODE_ID_VALUE 3)
else()
    message(FATAL_ERROR "Invalid HOME_NODE_ROLE='${HOME_NODE_ROLE}'. Use kitchen|living|bedroom.")
endif()

message(STATUS "HomeNode role: ${HOME_NODE_ROLE_LOWER} (node=${HOME_NODE_ID_VALUE})")

# Enable CMake support for ASM, C and C++ languages
enable_language(C CXX ASM)

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
    ${CMAKE_CURRENT_SOURCE_DIR}/App/Src/bsp_delay.c
    ${CMAKE_CURRENT_SOURCE_DIR}/App/Src/bsp_esp01s.c
    ${CMAKE_CURRENT_SOURCE_DIR}/App/Src/bsp_dht11.c
    ${CMAKE_CURRENT_SOURCE_DIR}/App/Src/bsp_motor.c
    ${CMAKE_CURRENT_SOURCE_DIR}/App/Src/bsp_relay.c
    ${CMAKE_CURRENT_SOURCE_DIR}/App/Src/app_node.c
    ${CMAKE_CURRENT_SOURCE_DIR}/App/Src/app_home_protocol.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/App/Inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    APP_NODE_ID=${HOME_NODE_ID_VALUE}
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

