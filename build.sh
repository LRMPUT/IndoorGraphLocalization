echo "Configure and build g2o"

cd EXTERNAL/g2o
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j2
cd ../../..

echo "Configure and build IndoorGraphLocalization"

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j2
