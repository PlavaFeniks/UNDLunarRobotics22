#include <pthread.h>
#include <iostream>
#include <filesystem>
#include <dirent.h>
#include <fcntl.h>
#include <fstream>
#include <stdlib.h>
#include <errno.h>
#include <chrono>
#include <thread>
#include "readSerial.h"
#include "../movement.h"


using namespace std;
struct thread_data{
    string * dataEntries; 
    TalonPair * motorControllers;
    readSerial * serialConnection;
};
bool islogging = false;

void * logging(void* threadData) {
    thread_data * t_data = (thread_data *)(threadData);
    string *logEntries = t_data->dataEntries;
    TalonPair *mc_s = t_data->motorControllers;
    
    if ( sizeof(logEntries) != 2* sizeof(mc_s) ) {
        std::_Exit(0);
    }
    

	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
	int count;
	DIR *logsDir;
	int i = 0;
	struct dirent *ep;
	logsDir = opendir("./logs");
	if (logsDir){
		while (ep = readdir(logsDir)){
			i++;
		}
		(void)closedir(logsDir);
	}
	
	string fileOpen = "./logs/log"+to_string(i)+".csv";
	std::ofstream outputCSV;
	outputCSV.open(fileOpen.c_str());
	cout<<fileOpen<<" has been opened for logging"<<endl;
    outputCSV<<"time,";
    for (int i = 0; i < sizeof(logEntries); i++){
        if(i == (sizeof(logEntries)-1)){
            outputCSV<<logEntries[i]<<"\n";
        }
        else{
            outputCSV<<logEntries[i]<<",";
        }
    }
	while(islogging)
	{


            
		end = std::chrono::steady_clock::now();
        outputCSV<<std::chrono::duration_cast<std::chrono::milliseconds> (end - begin).count()<<",";
        for( int i = 0; i< sizeof(mc_s); i++){
            outputCSV<<mc_s[i].getVoltage()<<","<<mc_s[i].getQuadVelocity();
            if( i == (sizeof(mc_s)- 1)){
                outputCSV<<"\n";
            }
            else{
                outputCSV<<",";
            }
        }
	this_thread::sleep_for(chrono::milliseconds(250));
	}
	
	
	outputCSV.close();
	pthread_exit(NULL);
	
}


/* move to ampSerial
void *loggingThread2(void *threadData){
	//thread for logging
	char aboslute_path[] = "/dev/ttyACM0";
	int serial_fd;
    serial_fd = open(aboslute_path, O_RDONLY|O_NOCTTY); //O_NONBLOCK
    if (serial_fd < 0) {
    printf("Error %i from open: %s\n", errno, strerror(errno));
    std::_Exit(0);
    }
   

    
        struct termios tty;
        char read_buf;


        cfsetspeed(&tty,B9600);
        if(tcgetattr(serial_fd, &tty) != 0) {
            printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        }   
        
        cfmakeraw(&tty);
        
        tty.c_cflag |= CS8;
        tty.c_lflag |=(CLOCAL| CREAD);
        tty.c_iflag &= ~(IXOFF|IXON);
        tty.c_cc[VMIN] = 1;
        tty.c_cc[VTIME] = 0;
        
        
        
       tcsetattr(serial_fd,TCSANOW,&tty);
       double current = 0.00;
       
	const auto start = std::chrono::system_clock::now();
	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
	int count;
	DIR *logsDir;
	int i = 0;
	struct dirent *ep;
	logsDir = opendir("./logs");
	if (logsDir){
		while (ep = readdir(logsDir)){
			i++;
		}
		(void)closedir(logsDir);
	}
	
	//outputCSV = fopen(fileOpen.c_str(), "w+"); //add screw back
	string fileOpen = "./logs/log"+to_string(i)+".csv";
	std::ofstream outputCSV;
	outputCSV.open(fileOpen.c_str());
	cout<<fileOpen<<" has been opened for logging"<<endl;
	outputCSV<<"time, rear talLeft voltage, rear talRght voltage , talLeft voltage, talrght voltage, screwdriver voltage, bucket voltage, hopper voltage, CURRENT(amps)\n";
	while(logging)
	{
			string temp;
            while(read_buf != ','){
                int n = read(serial_fd, &read_buf, 1);//was sizeof(readbuf)
                temp += read_buf;
                
            }   
            
            read_buf = ' ';
           // cout<<current<<endl;
            
            current = atof(temp.c_str());
		end = std::chrono::steady_clock::now();
		//Time_elapsed is actual code, rest is currently pseudo
		outputCSV<< std::chrono::duration_cast<std::chrono::milliseconds> (end - begin).count() <<","<< rear_talLeft.GetMotorOutputVoltage()<< "," << rear_talRght.GetMotorOutputVoltage() << ",";
		outputCSV<< talLeft.GetMotorOutputVoltage()<< "," <<talRght.GetMotorOutputVoltage() << ",";
		outputCSV<< screwDriver.GetMotorOutputVoltage()<<"," <<diggerDrive.GetMotorOutputVoltage()<<","; // add back screw_log.GetQuadraturePosition
		outputCSV<< hopper.GetMotorOutputVoltage()<<","<<to_string(current)<<"\n";
		sleepApp(250);
	}
	
	
	//ogging = true;
	cout<<"HHHHHHHHHHHHHHHHHHHHHHHHHHAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA\n";
	outputCSV.close();
	pthread_exit(NULL);
	
}
*/