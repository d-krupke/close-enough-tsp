add_executable(doctests ./main.cpp)
target_link_libraries(doctests PRIVATE doctest::doctest)
target_link_libraries(doctests PRIVATE  Gurobi::GurobiCXX)