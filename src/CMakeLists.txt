add_library(cetsp)
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