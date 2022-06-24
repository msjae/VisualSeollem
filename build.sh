echo "Configuring and building libglew-dev, libuvc ..."

sudo apt-get install libglew-dev
git clone http://github.com/ktossell/libuvc
cd libuvc
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make && sudo make install

cd ../../
python3 buildDeps.py

echo "Configuring and building Thirdparty/DBoW2 ..."

cd ../../Thirdparty/DBoW2
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j
sudo make install



echo "Configuring and building Thirdparty/g2o ..."
cd ../../g2o
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$nproc
sudo make install



echo "Configuring and building Thirdparty/Pangolin ..."
cd ../../Pangolin
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$nproc
sudo make install


echo "Uncompress vocabulary ..."
cd ../../
cd Vocabulary
tar -xf ORBvoc.txt.tar.gz


echo "Configuring and building ORB_SLAM2 ..."
cd ..
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$nproc
sudo make install
