#!/bin/bash



apt-get update

export temp_home=$(pwd)

#install Opencv
sudo apt-get install -y cmake g++ wget unzip black
cd ~/Downloads
wget -O ~/Downloads/opencv.zip https://github.com/opencv/opencv/archive/4.x.zip
wget -O ~/Downloads/opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/4.x.zip
unzip opencv.zip
unzip opencv_contrib.zip

mkdir -p ~/Downloads/build && cd ~/Downloads/build
cmake ../opencv-4.x
cmake --build
#return to original place 
#install sdl2
sudo apt-get install libsdl2-dev

#pipe input to  /etc/networks/interface file

sudo cp $temp_home/pi/interfaces.d  /etc/network/interfaces
sudo cp $temp_home/pi/robo.service /etc/systemd/system/
sudo systemctl daemon-reload && sudo systemctl enable robo.service && sudo systemctl start robo.service
sudo apt-get upgrade


#clone repository for CTRE's Phoenix Software

sudo git clone "git@github.com:CrossTheRoadElec/Phoenix-Linux-SocketCAN-Example.git" home/root/Downloads/
sudo python3 installPhoenix.py &

echo temp_home=""

echo "All required dependencies installed !!!\n"
echo "Enjoy"
