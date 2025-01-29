
Build with

```bash
conan install . --output-folder=build --build=missing
cmake . -DCMAKE_TOOLCHAIN_FILE=./build/conan_toolchain.cmake -DCMAKE_PREFIX_PATH=./.conan -DCMAKE_BUILD_TYPE=Release -B build
cmake --build build
```

Run tests with

```bash
./build/cetsp_solver/test_cetsp_solver
```