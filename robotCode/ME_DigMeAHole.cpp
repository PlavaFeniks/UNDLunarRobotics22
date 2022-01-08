#define Phoenix_No_WPI // remove WPI dependencies
#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include "ctre/phoenix/cci/Unmanaged_CCI.h"
#include <string>
#include <iostream>
#include <chrono>
#include <thread>
#include <SDL2/SDL.h>
#include <filesystem>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include <string>
#include <fstream>
#include <sys/types.h>
#include <dirent.h>
#include <stdio.h>
#include <termios.h>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
using namespace std;

using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;
TalonSRX backupDrive(5);
TalonSRX backupDig(6);
void sleepApp(int ms)
{
	std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

char aboslute_path[] = "/dev/ttyACM0";

class TalonPair{
    protected:
		
	public:
    TalonSRX *mc;
		SensorCollection* sc;
		TalonPair();
		void setSpeed(double);
        TalonPair(int canID){
        mc = new TalonSRX(canID);
        sc = &mc->GetSensorCollection();
    }
};
void TalonPair::setSpeed(double speed){
    mc->Set(ControlMode::PercentOutput, speed);

};

class ThrottledTalon: public TalonPair{
    protected:
        int maxMag;
    public:
        ThrottledTalon();
        ThrottledTalon(int canID,double max_MAG){
            mc = new TalonSRX(canID);
            sc = &mc->GetSensorCollection();
            maxMag = max_MAG;
        }
};

class tractioned_pair{
    private:
        bool t_enabled;
    public:
        ThrottledTalon *left;
        ThrottledTalon *right;
        tractioned_pair();
        tractioned_pair(int canID_l, int canID_r, double max_MAG, bool t_en){
            left = new ThrottledTalon(canID_l, max_MAG);
            right = new ThrottledTalon(canID_r,max_MAG);
            t_enabled = t_en;
        }
        void set_pair(double, double);
};

void tractioned_pair::set_pair(double speedL, double speedR){
    if(t_enabled){
        // Do calculations here
    }
    else{
        left->setSpeed(speedL);
        right->setSpeed(speedR);
    }
}

TalonPair screwDrive(5);
TalonPair diggerDrive(6);

int main(int argc, char *argv[]){ 

    



    double digSped,screwMeHarder;
    if(argc < 3){
        cout<<"Improper usage\n"<<flush;
        
        std::_Exit(0);
    }
    else if (argc == 4){
		//digSped = (float)(atof(argv[1]) / 100);
		digSped = 0.00;
		screwMeHarder = (float) (atof(argv[2])/100);
		cout<<digSped<<screwMeHarder<<flush;
		cout<<"HOLY FUCK ITS BEEN REVERSED\n"<<flush;
		backupDrive.SetInverted(false);
	}
    else{
		digSped = (float)(atof(argv[1]) / 100);
		screwMeHarder = (float) (atof(argv[2])/100);
		cout<<digSped<<screwMeHarder<<flush;
		backupDrive.SetInverted(true);
	}
    



    struct termios tty;
    int serial_fd;
    serial_fd = open(aboslute_path, O_RDONLY );


    cfsetspeed(&tty,B9600);
    if(tcgetattr(serial_fd, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    }   
    char read_buf[18];
    int n = read(serial_fd, &read_buf, sizeof(read_buf));
    double current = 0.0;



    
    

    string interface;
	interface = "can0";
	int temp; 
	if (temp = (ctre::phoenix::platform::can::SetCANInterface(interface.c_str())) == -1){
		perror("");
		std::_Exit(0);
	}
	sleep(1);

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    ofstream outStream;
    string fileName = "logs/output_" + to_string(digSped) + "_" + to_string(screwMeHarder) + ".csv";
    outStream.open(fileName.c_str());
    outStream<<"time, screwdrive voltage, extractor voltage, current"<<endl;

   ctre::phoenix::unmanaged::FeedEnable(100);

    //screwDrive.mc->SetInverted(true);
    //diggerDrive.mc->SetInverted(false);

	//backupDrive.SetInverted(false);
    while( int(std::chrono::duration_cast<std::chrono::seconds> (end - begin).count()) < 30){
       
        end = std::chrono::steady_clock::now();
        ctre::phoenix::unmanaged::FeedEnable(2000);
        backupDig.Set(ControlMode::PercentOutput, digSped); //scoops
        if ( (chrono::duration_cast<std::chrono::seconds>(end - begin).count() % 2) == 1) {
			backupDrive.Set(ControlMode::PercentOutput, 0.00f);
		}
		else backupDrive.Set(ControlMode::PercentOutput, screwMeHarder); //screwball
        //screwDrive.mc->Set(ControlMode::PercentOutput, digSped);
        //diggerDrive.mc->Set(ControlMode::PercentOutput, screwMeHarder);
		
        int n = read(serial_fd, &read_buf, sizeof(read_buf));
        current = atof(read_buf);
        //cout<<current<<endl;


        outStream<<std::chrono::duration_cast<std::chrono::seconds> (end - begin).count()<<",";
        outStream<<backupDrive.GetMotorOutputVoltage()<<",";
        outStream<<backupDig.GetMotorOutputVoltage()<<","<<current<<"\n";

        sleepApp(500);
		cout<<"yes\n";

    }
    outStream.close();
    begin = std::chrono::steady_clock::now();
    close(serial_fd);
    
    
    
    backupDrive.SetInverted(false);
    while (int(std::chrono::duration_cast<std::chrono::seconds>(end-begin).count())<30){
		ctre::phoenix::unmanaged::FeedEnable(2000);
		backupDrive.Set(ControlMode::PercentOutput, screwMeHarder);
		end = std::chrono::steady_clock::now();
	}
	
    return(0);
}
