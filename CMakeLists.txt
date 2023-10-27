# Minimum cmake version required
cmake_minimum_required(VERSION 3.8)

# Project configuration
project(PhysicsEngine LANGUAGES CXX)

# Install libraries into correct locations on all platforms.
include(GNUInstallDirs)

# Options
option(PHYSICSENGINE_COMPILE_TESTS "Build Tests" ON)
# option(PHYSICSENGINE_COMPILE_DEMO "Build Demos" OFF)

# Path to include and src directories
set(INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include")
set(SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")

# Search for header and source files to be included in library
file(GLOB_RECURSE PHYSICSENGINE_HEADERS "${INCLUDE_DIR}/*.h")
file(GLOB_RECURSE PHYSICSENGINE_SOURCES "${SRC_DIR}/*.cpp")

# Create library
add_library(physicsengine ${PHYSICSENGINE_HEADERS} ${PHYSICSENGINE_SOURCES})

# Create ALIAS
add_library(PhysicsEngine::physicsengine ALIAS physicsengine)

# C++11 compiler features
target_compile_features(physicsengine PUBLIC cxx_std_11)
set_target_properties(physicsengine PROPERTIES CXX_EXTENSIONS OFF)
add_compile_options(/W4)

# Library headers
target_include_directories(physicsengine PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)

# Compile tests
if(PHYSICSENGINE_COMPILE_TESTS)
  enable_testing()
  add_subdirectory(test/)
endif()

# # Compile demo
# if(PHYSICSENGINE_COMPILE_DEMO)
#   add_subdirectory(demo/)
# endif()

# Install target
install(TARGETS physicsengine EXPORT physicsengine-targets ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

# Install headers separately
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

set_target_properties(physicsengine PROPERTIES EXPORT_NAME PhysicsEngine)

install(EXPORT physicsengine-targets FILE PhysicsEngineConfig.cmake NAMESPACE PhyicsEngine:: DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/PhysicsEngine)