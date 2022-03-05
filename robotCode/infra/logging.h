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
};

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
	while(logging)
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
	
	
	//ogging = true;
	outputCSV.close();
	pthread_exit(NULL);
	
}



}