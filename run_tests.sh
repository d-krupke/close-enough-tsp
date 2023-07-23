conan install . --build=missing --output-folder=.conan -g CMakeDeps -g CMakeToolchain -s build_type=Debug &&\
 mkdir -p build && cmake . -DCMAKE_TOOLCHAIN_FILE=../.conan/conan_toolchain.cmake -DCMAKE_PREFIX_PATH=../.conan -DCMAKE_BUILD_TYPE=Debug -B build &&\
  cmake --build build --target doctests -- -j 12 && pip install . && ./build/tests/doctests && pytest
