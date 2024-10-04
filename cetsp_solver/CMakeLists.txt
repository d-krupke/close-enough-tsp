#add_library(cetsp_solver STATIC)

find_package(gurobi REQUIRED)
#==============================================================================
# Library: cetsp_solver
#==============================================================================

add_library(cetsp_solver STATIC
./src/soc.cpp
./src/data.cpp
)
target_include_directories(cetsp_solver PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include/")
target_link_libraries(cetsp_solver PUBLIC gurobi::gurobi)

#==============================================================================
# Testing with Google Test
#==============================================================================

# Glob all the test source files recursively under the "tests" directory
# FOR NEW PROJECTS: Make sure to add new test files to the "tests" directory or
#                   change the directory name in this file.
file(GLOB_RECURSE TEST_SOURCES "tests/*.cpp")

# Create an executable named "test_cetsp_solver" using the test source files
# FOR NEW PROJECTS: Choose a different name for the test executable. Find and 
#                   replace "test_cetsp_solver" in this file.
add_executable(test_cetsp_solver ${TEST_SOURCES})

# Link the cetsp_solver library to the test executable
# FOR NEW PROJECTS: Make sure to link all the libraries that the test executable
#                   depends on.
target_link_libraries(test_cetsp_solver PUBLIC cetsp_solver)
target_link_libraries(test_cetsp_solver PUBLIC gurobi::gurobi)

# Link the GTest library to the test executable. This will automatically
# create the main function that runs all the tests, so you don't have to
# write it yourself (as done in some tutorials).
target_link_libraries(test_cetsp_solver PUBLIC GTest::gtest_main)

# Set compiler options for the test executable. These are not required, but
# they can help catch bugs and enforce a consistent coding style.
target_compile_options(
    test_cetsp_solver PRIVATE "$<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wall>")
target_compile_options(
    test_cetsp_solver PRIVATE "$<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wpedantic>")
target_compile_options(
    test_cetsp_solver PRIVATE "$<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wextra>")

# Include the GoogleTest module, so we can use the gtest_discover_tests function
include(GoogleTest)

# Discover and run tests using GoogleTest
gtest_discover_tests(test_cetsp_solver
    # Set a working directory for the test executable. This is required if the
    # test executable reads files from the disk.
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/tests"
)
#==============================================================================