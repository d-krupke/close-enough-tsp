add_executable(
  doctests
  ./main.cpp
  ../include/cetsp/details/cgal_kernel.h
  ../include/cetsp/common.h
  ../include/cetsp/soc.h
  ../src/soc.cpp
  ../src/geometry.cpp
  ../include/cetsp/heuristics.h
  ../src/heuristics.cpp
  ../src/node.cpp
  ../include/cetsp/node.h
  ../include/cetsp/bnb.h
  ../include/cetsp/strategies/root_node_strategy.h
  ../include/cetsp/details/solution_pool.h
  ../include/cetsp/strategies/branching_strategy.h
  ../include/cetsp/strategies/search_strategy.h
  ../include/cetsp/callbacks.h
  ../src/root_node_strategies/convex_hull_root.cpp
  ../src/branching_strategy.cpp
  ../include/cetsp/details/triple_map.h
  ../include/cetsp/details/convex_hull_order.h
  ../src/convex_hull_order.cpp
  ../src/relaxed_solution.cpp
  ../include/cetsp/details/lazy_trajectory.h
  ../src/root_node_strategies/longest_edge_plus_farthest_circle.cpp
  ../src/branching_strategies/global_convex_hull.cpp
  ../src/branching_strategies/layered_convex_hull_rule.cpp
  ../include/cetsp/strategies/rule.h
  ../include/cetsp/strategies/rules/global_convex_hull_rule.h
  ../include/cetsp/details/missing_disks_lb.h
  lazy_callback_tests.h)
target_include_directories(doctests PRIVATE ../include)
target_link_libraries(doctests PRIVATE doctest::doctest)
target_link_libraries(doctests PRIVATE ${cgal_LIBRARIES})
target_link_libraries(doctests PRIVATE gurobi::gurobi)
target_compile_options(
  doctests PRIVATE "$<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wall>")
target_compile_options(
  doctests PRIVATE "$<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wpedantic>")
target_compile_options(
  doctests PRIVATE "$<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wextra>")
