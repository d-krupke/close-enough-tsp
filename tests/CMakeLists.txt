add_executable(doctests ./main.cpp
        ../include/cetsp/cgal_kernel.h
        ../include/cetsp/common.h
        ../include/cetsp/soc.h
        ../src/soc.cpp
        ../include/cetsp/heuristics.h ../src/heuristics.cpp)
target_include_directories(doctests PRIVATE ../include)
target_link_libraries(doctests PRIVATE doctest::doctest)
target_link_libraries(doctests PRIVATE CGAL::CGAL)
target_link_libraries(doctests PRIVATE gurobi::gurobi)
#target_link_libraries(doctests PRIVATE  Gurobi::GurobiCXX)