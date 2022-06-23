echo "Configuring and building libglew-dev, libuvc ..."

sudo apt-get install libglew-dev
git clone http://github.com/ktossell/libuvc
cd libuvc
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make && sudo make install

echo "Configuring and building Thirdparty/DBoW2 ..."

cd ../../Thirdparty/DBoW2
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j
sudo make install

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$nproc
sudo make install

cd ../../Pangolin

echo "Configuring and building Thirdparty/Pangolin ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$nproc
sudo make install

cd ../../easy_profiler

echo "Configuring and building Thirdparty/easy_profiler ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$nproc
sudo make install

cd ../../opencv

echo "Configuring and building Thirdparty/opencv ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$nproc
sudo make install

cd ../../../


echo "Uncompress vocabulary ..."

cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..

echo "Configuring and building ORB_SLAM2 ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$nproc
sudo make install
