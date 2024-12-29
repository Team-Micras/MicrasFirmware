bash docker/scripts/cube.sh

mkdir -p build
cd build
cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=OFF ..
make -j
