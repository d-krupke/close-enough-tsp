add_executable(profiling profiling.cpp)
target_include_directories(profiling PRIVATE ../include)
target_compile_definitions(profiling PRIVATE DOCTEST_CONFIG_DISABLE)
target_link_libraries(profiling PRIVATE doctest::doctest)
target_link_libraries(profiling PRIVATE CGAL::CGAL)
target_link_libraries(profiling PRIVATE gurobi::gurobi)
target_link_libraries(profiling PRIVATE cetsp)
target_compile_options(profiling PRIVATE
        "$<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wall>")
target_compile_options(profiling PRIVATE
        "$<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wpedantic>")
target_compile_options(profiling PRIVATE
        "$<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wextra>")
