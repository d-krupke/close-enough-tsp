add_library(cetsp ../include/cetsp/heuristics.h heuristics.cpp node.cpp ../include/cetsp/node.h ../include/cetsp/bnb.h ../include/cetsp/root_node_strategy.h ../include/cetsp/solution_pool.h ../include/cetsp/branching_strategy.h)
target_sources(cetsp
        PUBLIC
        ../include/cetsp/common.h
        ../include/cetsp/cgal_kernel.h
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
target_compile_definitions(cetsp PRIVATE DOCTEST_CONFIG_DISABLE)