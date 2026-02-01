cmake_minimum_required(VERSION 3.15)
project(sesx_import VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Source files
set(SOURCES
    src/main.cpp
    include/pugixml/pugixml.cpp
)

# Create the shared library
add_library(${PROJECT_NAME} SHARED ${SOURCES})

# Include directories
target_include_directories(${PROJECT_NAME} PRIVATE
    include/reaper_sdk
    include/pugixml
)

# Reaper plugins have specific naming conventions based on the platform.
if(APPLE)
    set_target_properties(${PROJECT_NAME} PROPERTIES
        PREFIX "reaper_"
        SUFFIX "-arm64.dylib"
    )
elseif(WIN32)
    set_target_properties(${PROJECT_NAME} PROPERTIES
        PREFIX "reaper_"
        SUFFIX "-x64.dll"
    )
endif()

# Automated deployment to Reaper UserPlugins
if(APPLE)
    set(REAPER_USER_PLUGINS "$ENV{HOME}/Library/Application Support/REAPER/UserPlugins")
elseif(WIN32)
    set(REAPER_USER_PLUGINS "$ENV{APPDATA}/REAPER/UserPlugins")
endif()

if(REAPER_USER_PLUGINS)
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory "${REAPER_USER_PLUGINS}"
        COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${PROJECT_NAME}> "${REAPER_USER_PLUGINS}/$<TARGET_FILE_NAME:${PROJECT_NAME}>"
        COMMENT "Copying ${PROJECT_NAME} to ${REAPER_USER_PLUGINS}"
    )
endif()

message(STATUS "Build configuration: ${CMAKE_BUILD_TYPE}")
