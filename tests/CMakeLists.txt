add_executable(doctests ./main.cpp
        ../include/cetsp/details/cgal_kernel.h
        ../include/cetsp/common.h
        ../include/cetsp/soc.h
        ../src/soc.cpp
        ../include/cetsp/heuristics.h ../src/heuristics.cpp ../src/node.cpp ../include/cetsp/node.h ../include/cetsp/bnb.h ../include/cetsp/details/root_node_strategy.h ../include/cetsp/details/solution_pool.h ../include/cetsp/details/branching_strategy.h ../include/cetsp/details/search_strategy.h ../include/cetsp/callbacks.h)
target_include_directories(doctests PRIVATE ../include)
target_link_libraries(doctests PRIVATE doctest::doctest)
target_link_libraries(doctests PRIVATE CGAL::CGAL)
target_link_libraries(doctests PRIVATE gurobi::gurobi)
#target_link_libraries(doctests PRIVATE  Gurobi::GurobiCXX)