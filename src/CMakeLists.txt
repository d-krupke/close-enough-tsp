add_library(cetsp ../include/cetsp/heuristics.h heuristics.cpp node.cpp ../include/cetsp/node.h ../include/cetsp/bnb.h ../include/cetsp/details/root_node_strategy.h ../include/cetsp/details/solution_pool.h ../include/cetsp/details/branching_strategy.h ../include/cetsp/details/search_strategy.h ../include/cetsp/callbacks.h root_node_strategies/convex_hull_root.cpp branching_strategy.cpp ../include/cetsp/details/triple_map.h ../include/cetsp/details/convex_hull_order.h convex_hull_order.cpp ../include/cetsp/utils/timer.h ../include/cetsp/relaxed_solution.h ../include/cetsp/details/distance_cache.h ../include/cetsp/utils/lazy_member.h relaxed_solution.cpp ../include/cetsp/details/lazy_trajectory.h geometry.cpp root_node_strategies/longest_edge_plus_farthest_circle.cpp)
target_sources(cetsp
        PUBLIC
        ../include/cetsp/common.h
        ../include/cetsp/details/cgal_kernel.h
        ../include/cetsp/soc.h
        PRIVATE
        ./soc.cpp)
target_include_directories(cetsp PUBLIC ../include)
target_compile_options(cetsp PRIVATE
        "$<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wall>")
set_target_properties(cetsp PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(cetsp PRIVATE gurobi::gurobi)
target_link_libraries(cetsp PUBLIC CGAL::CGAL)
target_link_libraries(cetsp PUBLIC doctest::doctest)
target_link_libraries(cetsp PRIVATE ${Boost_LIBRARIES})
target_compile_definitions(cetsp PRIVATE DOCTEST_CONFIG_DISABLE)
