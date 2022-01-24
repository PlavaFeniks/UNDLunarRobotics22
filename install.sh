#!/bin/bash



apt-get update

export temp_home=$(pwd)

#install Opencv
sudo apt-get install -y cmake g++ wget unzip
cd ~/Downloads
wget -O ~/Downloads/opencv.zip https://github.com/opencv/opencv/archive/4.x.zip
unzip  ~/Downloads/opencv.zip

mkdir -p ~/Downloads/build && cd ~/Downloads/build
cmake ../opencv-4.x
cmake --build
#return to original place 
#install sdl2
sudo apt-get install libsdl2-2.0



#pipe input to  /etc/networks/interface file

sudo cp $temp_home/pi/interfaces.d  /etc/network/
sudo cp $temp_home/pi/robo.service /etc/systemd/system/
sudo systemctl daemon-reload && sudo systemctl enable robo.service && sudo systemctl start robo.service
sudo apt-get upgrade

echo temp_home=""

echo "All required dependencies installed !!!\n"
echo "Enjoy"
