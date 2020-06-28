
COMTYPE=Release

#build thirdparty lib
cd Thirdparty/g2o
#g2o
#echo "build g2o ..."
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=$COMTYPE .. 

cd ../../../
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=$COMTYPE .. 
make -j4
#../output/Positioning
