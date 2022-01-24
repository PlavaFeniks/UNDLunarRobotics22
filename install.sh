#!/bin/bash



apt-get update



#install Opencv
apt-get install -y cmake g++ wget unzip
wget -O opencv.zip https://github.com/opencv/archive/4.x.zip ~/Downloads
unzip  ~/Downloads/opencv.zip
mkdir -p ~/Downloads/build && cd ~/Downloads/build
cmake ../opencv-4.x
cmake --build
#install sdl2
apt-get install libsdl2-2.0



#pipe input to  /etc/networks/interface file
cat pi/interfaces.d >> /etc/network/interfaces
cp pi/robo.service /etc/systemd/system/
systemctl daemon-reload && systemctl enable robo.service && systemctl start robo.service
apt-get updgrade

echo "All required dependencies installed !!!\n"
echo "Enjoy"
