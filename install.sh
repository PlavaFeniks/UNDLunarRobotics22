#!/bin/bash


apt-get update
o_flag=false
export temp_home=$(pwd)
while getopts po flag; 
do 
    case "${flag}" in
        p) sudo python3 $temp_home/pi/fix_sudo-apt.python3 ;;
        o)  echo "opencv will be installed"
            o_flag=true 
            ;;
    esac
done 

#install Opencv
sudo apt-get install -y cmake g++ wget unzip black libsdl2-dev
if $o_flag;
then
    cd ~/Downloads

    wget -O ~/Downloads/opencv.zip https://github.com/opencv/opencv/archive/4.x.zip
    wget -O ~/Downloads/opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/4.x.zip
    unzip opencv.zip
    unzip opencv_contrib.zip

    mkdir -p ~/Downloads/build && cd ~/Downloads/build
    cmake -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib-4.x/modules ../opencv-4.x
    cmake --build .
else 
    echo "Re-run command with -o to install opencv"
fi

#pipe input to  /etc/networks/interface file
sudo cp $temp_home/pi/interfaces  /etc/network/interfaces
sudo cp $temp_home/pi/robo.service /etc/systemd/system/
cd $temp_home/robotCode && cmake . && make
sudo systemctl daemon-reload && sudo systemctl enable robo.service && sudo systemctl start robo.service
sudo apt-get upgrade


#clone repository for CTRE's Phoenix Software < this may result in errors because CTRE likes to update cmake and shared objects>

sudo git clone "https://github.com/CrossTheRoadElec/Phoenix-Linux-SocketCAN-Example.git" /tmp/Downloads/Phoenix-Linux-SocketCAN-Example
sudo python3 installPhoenix.py &

echo temp_home=""

echo "All required dependencies installed !!!\n"
echo "Enjoy"
