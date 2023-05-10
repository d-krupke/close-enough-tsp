# This is the root file for CMake, which is used to define the native part of
# the project. It is completely independent of the Python-part and can actually
# be used just to build a C++-library.
cmake_minimum_required(VERSION 3.23)
option(CXX "enable C++ compilation" ON)

if(CXX)
  enable_language(CXX)
endif()
project(cetsp CXX) # Feel free to add more project information
set(CMAKE_CXX_STANDARD 17) # Using C++-17 (most features with good support at
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
cpmaddpackage("gh:doctest/doctest@2.4.9")
# further CPM-dependencies are defined in the subdirectories

# ~~~
# Conan dependencies ~~~~~~~~~~~~~~~~~~~~~
# We are using the `cmake_find_package` generator to create `FindXXX`-files
# in the `cmake`-folder. This is only necessary, if the packages are not
# already installed. This approach is not without its problems:
# PRO: Simple option that is compatible with scikit-build. Can work without conan.
# CON: Difficult if you want to use changing conan settings.

# run `conan install .. --build=missing` in `.conan`.
#list(APPEND CMAKE_PREFIX_PATH "${CMAKE_SOURCE_DIR}/.conan")
# Also need to be defined in `./conanfile.txt`.
find_package(nlohmann_json REQUIRED)
find_package(CGAL REQUIRED)
find_package(gurobi REQUIRED)
include_directories(${GUROBI_INCLUDE_DIRS})
#find_package(Boost REQUIRED COMPONENTS thread system)

# ~~~
# System dependencies ~~~~~~~~~~~~~~~~~~~~~~
# These files have to be installed on the system
# set(GUROBI_HOME "/Library/gurobi1000/macos_universal2/")
# find_package(GUROBI REQUIRED)

# ~~~
# C++-Library ~~~~~~~~~~~~~~~~~~~~~~~~~
# This part defines the actual C++-library that may be used as a simple
# C++-library. The Python-interface to this library is defined separately.
add_subdirectory(src)

# ~~~
# TESTS OF THE PUBLIC INTERFACE ~~~~~~~~~~~~~~
# A good programmer will add at least some unit tests.
# The tests-folder can also be used for Python-tests, without interference.
if(NOT SKBUILD)
  # only import if exists. The production code may be shipped without.
  add_subdirectory(tests) # defined in a separate CMakeLists.txt
endif()

# ~~~
# Python Interface ~~~~~~~~~~~~~~~~~~~~
# The Python-bindings are compiled as separate target to allow the independent
# usage of the C++-library.
add_subdirectory(pysrc) # defined in a separate CMakeLists.txt

# ~~~
# Apps ~~~~~~~~~~~~~~~~~~~~~~~~~~~
if(NOT SKBUILD) # only make available if not using SKBUILD.
  # define your apps here
  add_subdirectory(apps)
endif()
