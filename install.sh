#!/bin/bash



apt-get update

export temp_home=$(pwd)
export num_procs=$(nproc)
#install Opencv
sudo apt-get install -y cmake g++ wget unzip black libsdl2-dev
cd ~/Downloads


#establish build directory
mkdir -p ~/Downloads/opencv_build/build && cd ~/Downloads/opencv_build
git clone https://github.com/opencv/opencv.git
git clone https://github.com/opencv/opencv_contrib.git

cd ~/Downloads/opencv_build/build
cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D INSTALL_C_EXAMPLES=ON \
    -D OPENCV_GENERATE_PKGCONFIG=ON \
    -D OPENCV_EXTRA_MODULES_PATH=~/opencv_build/opencv_contrib/modules \
    -D BUILD_EXAMPLES=ON ..

make -j$nproc
sudo make install

#pipe input to  /etc/networks/interface file

sudo cp $temp_home/pi/interfaces.d  /etc/network/interfaces
sudo cp $temp_home/pi/robo.service /etc/systemd/system/
sudo systemctl daemon-reload && sudo systemctl enable robo.service && sudo systemctl start robo.service
sudo apt-get upgrade


#clone repository for CTRE's Phoenix Software

sudo git clone "https://github.com/CrossTheRoadElec/Phoenix-Linux-SocketCAN-Example.git" home/root/Downloads/
sudo python3 installPhoenix.py &

echo temp_home=""

echo "All required dependencies installed !!!\n"
echo "Enjoy"
