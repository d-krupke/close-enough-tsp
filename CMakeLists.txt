# This is the root file for CMake, which is used to define the native part of
# the project. It is completely independent of the Python-part and can actually
# be used just to build a C++-library.
cmake_minimum_required(VERSION 3.23)
option(CXX "enable C++ compilation" ON)

if(CXX)
  enable_language(CXX)
endif()
project(cetsp CXX) # Feel free to add more project information
set(CMAKE_CXX_STANDARD 20) # Using C++-17 (most features with good support at
                           # 2022)
set(CMAKE_POSITION_INDEPENDENT_CODE ON) # The code needs to be compiled as PIC
                                        # to build the shared lib for python.
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
include(./cmake/CCache.cmake) # Magic to speed up compiling by caching.

# ~~~
# CPM Dependencies ~~~~~~~~~~~~~~~~
# CPM allows us to include some simple dependencies without much fuss.
# Great default, for everything more complicated, uses conan.
# Why not always use conan? CPM works without any interaction (conan needs
# to be called externally). CPM also has no problems with install PyBind11
# (which conan has at the time).
# If the dependency has complicated dependencies on its own or is slow to
# compile, better use conan.
# Check out https://github.com/cpm-cmake/CPM.cmake
include(./cmake/CPM.cmake) # Package manager for simple requirements.

cpmaddpackage("gh:fmtlib/fmt#9.1.0") # fmt for nice strings
CPMAddPackage(
  NAME googletest
  GITHUB_REPOSITORY google/googletest
  GIT_TAG v1.14.0
  VERSION 1.14.0
  OPTIONS
      "gtest_force_shared_crt ON"
)
# further CPM-dependencies are defined in the subdirectories

# ~~~
# Conan dependencies ~~~~~~~~~~~~~~~~~~~~~
#find_package(nlohmann_json REQUIRED)
#find_package(CGAL REQUIRED)
find_package(gurobi CONFIG REQUIRED)
#find_package(NLopt REQUIRED)
#include_directories(${GUROBI_INCLUDE_DIRS})
# find_package(Boost REQUIRED COMPONENTS thread system)

# ~~~
# System dependencies ~~~~~~~~~~~~~~~~~~~~~~
# These files have to be installed on the system
# set(GUROBI_HOME "/Library/gurobi1000/macos_universal2/")
# find_package(GUROBI REQUIRED)

# ~~~
# C++-Library ~~~~~~~~~~~~~~~~~~~~~~~~~
# This part defines the actual C++-library that may be used as a simple
# C++-library. The Python-interface to this library is defined separately.
add_subdirectory(cetsp_solver)
