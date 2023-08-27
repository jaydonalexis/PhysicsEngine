# Minimum cmake version required
cmake_minimum_required(VERSION 3.8)

# Project configuration
project(Tests)

# Install libraries into correct locations on all platforms.
include(GNUInstallDirs)
include(GoogleTest)

# Hides options
mark_as_advanced(FORCE BUILD_GMOCK BUILD_GTEST)
mark_as_advanced(FORCE INSTALL_GMOCK INSTALL_GTEST)
mark_as_advanced(FORCE gmock_build_tests)
mark_as_advanced(FORCE gtest_build_samples gtest_build_tests gtest_disable_pthreads gtest_hide_internal_symbols)
mark_as_advanced(FORCE gtest_force_shared_crt)
mark_as_advanced(FORCE CMAKE_DEBUG_POSTFIX)

set(BUILD_GMOCK ON)
set(INSTALL_GTEST OFF CACHE BOOL "Install googletest.")
set(INSTALL_GMOCK OFF CACHE BOOL "Install googlemock.")
set(gtest_hide_internal_symbols OFF CACHE BOOL "")

# Don't build gtest shared to ensure GTEST_FLAG_SET symbols can be resolved
set(BUILD_SHARED_LIBS OFF)

# Prevent overriding the parent project's compiler/linker settings on Windows
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

# Add subdirectory to build
add_subdirectory(googletest EXCLUDE_FROM_ALL)

file(GLOB Test_SRCS *.cpp)

add_executable(tests ${Test_SRCS})

target_link_libraries(tests PhysicsEngine::physicsengine GTest::gtest)

gtest_discover_tests(tests)

install(TARGETS tests RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})