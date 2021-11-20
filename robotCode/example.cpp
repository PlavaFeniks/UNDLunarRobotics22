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

#define upperDlim .5
#define lowerDlim .0
#define locoThresh .1
#define minHeight -500
#define maxHeight 500  
using namespace std;
using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

//Declare Prototypes please
void sleepApp(int ms);

//Declare filePointer//
//FILE *outputCSV;



/* make some talons for drive train */
TalonSRX diggerDrive(6);


TalonSRX talLeft(3);
TalonSRX talRght(4);
TalonSRX screwDriver(5);
TalonSRX hopper(7);

TalonSRX rear_talLeft(1);
TalonSRX rear_talRght(2);

SensorCollection diggSensor(diggerDrive);
SensorCollection r_tL(rear_talLeft);
SensorCollection r_tR(rear_talRght);
SensorCollection tL(talLeft);
SensorCollection tR(talRght);
SensorCollection screw_log(screwDriver);
SensorCollection hop(hopper);

double diggerDSpeed;
int screwHeight = 0;
bool logging = false;
//TalonSRX diggerDrive(6);


void initDrive()
{
	
}
void invertDrive(TalonSRX* sender)
{
	if(sender->GetInverted()){
		sender->SetInverted(false);
		return;
	}
	sender->SetInverted(true);
	
}
void screwDrive(double speed){
	/*
	if (!(screwHeight >= maxHeight or screwHeight <= minHeight) and speed != 0.00)
	{
	
	//screwHeight += 1;
	}
	*/
	screwDriver.Set(ControlMode::PercentOutput,speed);
	
	return;
}
void hoppinout(double speed){
	
	hopper.Set(ControlMode::PercentOutput,speed);
	return;
}
void stepDigger(double stepFunc){
	double tSpeed = 0;
	tSpeed = diggerDSpeed + stepFunc;
	if ((tSpeed >= upperDlim) or (tSpeed<=lowerDlim)){
		std::cout<<"THIS IS AN INVALID NUMBER CANNOT GO ABOVE .85 or BELOW .10"<<std::endl;
		return;
	}
	else{
		diggerDSpeed = tSpeed;
		std::cout<<"DIGGER SPEED PERCENT CHANGED: "<<diggerDSpeed<<std::endl;
		sleepApp(500);
		return;
		
	}
}
void setDiggerDrive(){
	diggerDrive.Set(ControlMode::PercentOutput, diggerDSpeed);

	std::cout<<typeid(diggerDrive).name()<<std::endl;
	std::cout<<diggerDrive.GetSensorCollection().GetQuadraturePosition()<<"\nCOCKNBALLTORTURE\n";
	std::cout<<diggerDrive.GetMotorOutputVoltage()<<"\n";
	return;
}
void ldrive(double fwd, double turn)
{
	//double drivePerc = 1.00;
	double left = fwd - turn;
	//double rght = fwd + turn; /* positive turn means turn robot LEFT */

	//ctre::phoenix::unmanaged::FeedEnable(100);
	//std::cout<<ctre::phoenix::unmanaged::GetEnableState();
	talLeft.Set(ControlMode::PercentOutput, left);
	rear_talLeft.Set(ControlMode::PercentOutput, left);
	
	/*
	rear_talRght.Set(ControlMode::PercentOutput, rght);
	talRght.Set(ControlMode::PercentOutput, rght);
	*/
	
}

void rdrive(double fwd, double turn)
{
	//double drivePerc = 1.00;
	//double left = fwd - turn
	double rght = fwd + turn; /* positive turn means turn robot LEFT */

	//ctre::phoenix::unmanaged::FeedEnable(100);
	//std::cout<<ctre::phoenix::unmanaged::GetEnableState();
	rear_talRght.Set(ControlMode::PercentOutput, rght);
	talRght.Set(ControlMode::PercentOutput, rght);
	
	/*
	 * talLeft.Set(ControlMode::PercentOutput, left);
	rear_talLeft.Set(ControlMode::PercentOutput, left);
	
	*/
	
}
void drive(double fwd){
	
	//diggerDrive.Set(ControlMode::PercentOutput, fwd);
	
	
	
}
/* simple wrapper for code cleanup */
void sleepApp(int ms)
{
	std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

void *loggingThread2(void *threadData)
{
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
	outputCSV<<"time, rear talLeft voltage, rear talRght voltage , talLeft voltage, talrght voltage, screwdriver voltage, bucket voltage, hopper voltage\n";
	while(logging)
	{
		end = std::chrono::steady_clock::now();
		//Time_elapsed is actual code, rest is currently pseudo
		outputCSV<< std::chrono::duration_cast<std::chrono::seconds> (end - begin).count() <<","<< rear_talLeft.GetMotorOutputVoltage()<< "," << rear_talRght.GetMotorOutputVoltage() << ",";
		outputCSV<< talLeft.GetMotorOutputVoltage()<< "," <<talRght.GetMotorOutputVoltage() << ",";
		outputCSV<< screwDriver.GetMotorOutputVoltage()<<"," <<diggerDrive.GetMotorOutputVoltage()<<","; // add back screw_log.GetQuadraturePosition
		outputCSV<< hopper.GetMotorOutputVoltage()<<"\n";
		sleepApp(250);
	}
	
	
	//ogging = true;
	cout<<"HHHHHHHHHHHHHHHHHHHHHHHHHHAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA\n";
	outputCSV.close();
	pthread_exit(NULL);
	
}

int main() {
	/* don't bother prompting, just use can0 */
	//std::cout << "Please input the name of your can interface: ";
	
	pthread_t myThread;
	
	//pthread_join(myThread, NULL);
	
	std::string interface;
	
	//std::cin >> interface;
	interface = "can0";
	int temp; 
	if (temp = (ctre::phoenix::platform::can::SetCANInterface(interface.c_str())) == -1){
		perror("");
		std::_Exit(0);
	}
	
	// Comment out the call if you would rather use the automatically running diag-server, note this requires uninstalling diagnostics from Tuner. 
	// c_SetPhoenixDiagnosticsStartTime(-1); // disable diag server, instead we will use the diag server stand alone application that Tuner installs

	/* setup drive */
	initDrive();
	

	while (true) {
		/* we are looking for gamepad (first time or after disconnect),
			neutral drive until gamepad (re)connected. */
		ldrive(0, 0);
		rdrive(0, 0);
		diggerDSpeed = .25;
		
	

		// wait for gamepad
		printf("Waiting for gamepad...\n");
		while (true) {
			/* SDL seems somewhat fragile, shut it down and bring it up */
			SDL_Quit();
            SDL_SetHint(SDL_HINT_NO_SIGNAL_HANDLERS, "1"); //so Ctrl-C still works
			SDL_Init(SDL_INIT_JOYSTICK);

			/* poll for gamepad */
			int res = SDL_NumJoysticks();
			if (res > 0) { break; }
			if (res < 0) { printf("Err = %i\n", res); }

			/* yield for a bit */
			sleepApp(20);
		}
		printf("Waiting for gamepad...Found one\n");

		// Open the joystick for reading and store its handle in the joy variable
		SDL_Joystick *joy = SDL_JoystickOpen(0);
		if (joy == NULL) {
			/* back to top of while loop */
			continue;
		}
		
		// Get information about the joystick
		const char *name = SDL_JoystickName(joy);
		const int num_axes = SDL_JoystickNumAxes(joy);
		const int num_buttons = SDL_JoystickNumButtons(joy);
		const int num_hats = SDL_JoystickNumHats(joy);
		printf("Now reading from joystick '%s' with:\n"
			"%d axes\n"
			"%d buttons\n"
			"%d hats\n\n",
			name,
			num_axes,
			num_buttons,
			num_hats);

		

		// Keep reading the state of the joystick in a loop
		while (true) {
			/* poll for disconnects or bad things */
			//printf("WHAT THE FUCK");
			SDL_Event event;
			if (SDL_PollEvent(&event)) {
				if (event.type == SDL_QUIT) { break; }
				if (event.jdevice.type == SDL_JOYDEVICEREMOVED) { break; }
			}

			/* grab some stick values */
			double y = ((double)SDL_JoystickGetAxis(joy, 0)) / -32767.0;
			double turn = ((double)SDL_JoystickGetAxis(joy, 1)) / -32767.0;
			double ry = ((double)SDL_JoystickGetAxis(joy, 3)) / -32767.0;
			double rturn = ((double)SDL_JoystickGetAxis(joy, 4)) / -32767.0;
			diggerDrive.Set(ControlMode::PercentOutput, 0.00);

			for (int i = 0; i< num_buttons; i++){
				if (SDL_JoystickGetButton(joy,i)){
					cout<<"This is button " << i << "\n";
				}
			}
			
			//std::cout<<ry<<std::endl;
			//ctre::phoenix::unmanaged::FeedEnable(100);
			ldrive(0.00, 0.00);
			rdrive(0.00, 0.00);
			screwDrive(0.00);
			hoppinout(0.00);
			if(abs(turn) > locoThresh or abs(rturn) > locoThresh){
			//std::cout<< "ThisShouldBeMoving"<<std::endl;
			ldrive(y, turn);
			rdrive(ry, rturn);
			}
			
			if ((double)SDL_JoystickGetAxis(joy,2) > 0){
			setDiggerDrive();
			std::cout<<"WHY AM I RUNNING HOLY FUCK JESUS FUCKING CHRIST\n";
			
			}
			if ((double)SDL_JoystickGetAxis(joy,5) > 0){
				hoppinout(.50);
				std::cout<<"This Code is Dog Shit"<<std::endl;
			}
			//double q = ((double)SDL_JoystickGetAxis(joy,2))  / -32767.0;
			//drive(q);

			/* [SAFETY] only enable drive if top left shoulder button is held down */
			if (SDL_JoystickGetButton(joy, 4)) {
				ctre::phoenix::unmanaged::FeedEnable(100);
			}
			else if (SDL_JoystickGetButton(joy, 5)) {
				invertDrive(&diggerDrive);
				invertDrive(&hopper);
				sleepApp(500);
				
			}
			/*
			for (int i = 0; i<num_buttons;i++){
				if(SDL_JoystickGetButton(joy,i)){
				std::cout<<"EYO G THIS FUCKER IS BUTTON NUMBER: "<< i<<std::endl;
				}
			}
			*/
			
			if (SDL_JoystickGetButton(joy,2)){
				
				//cout<< n<<endl;
				if (!logging){
				logging = true;
				pthread_create(&myThread,NULL,loggingThread2,NULL);
				sleep(5);
				}
			}
			else if(SDL_JoystickGetButton(joy,3)){
				if(logging){
				logging = false;
				cout<<"EYO THE LOGGING THREAD HAS DONE BEEN KILLED\n";
				if(pthread_join(myThread, NULL) == 0)
				cout<<"Thread has successfully been joined, no longer a zombozo\n";
				sleep(5);
				}
			}
		
			if (SDL_JoystickGetHat(joy,0) == SDL_HAT_UP){
				std::cout<<"EVERYBODY WALK THE DINOSAUR"<<std::endl;
				stepDigger(.05);
			}
			else if (SDL_JoystickGetHat(joy,0) == SDL_HAT_DOWN){
				std::cout<<"EVERYBODY Get on the floor"<<std::endl;
				stepDigger(-.05);
			}
			else if (SDL_JoystickGetHat(joy,0) == SDL_HAT_LEFT){
				std::cout<<"EVERYBODY WALK THE DINOSAUR"<<std::endl;
				screwDrive(.50);
			}
			else if (SDL_JoystickGetHat(joy,0) == SDL_HAT_RIGHT){
				std::cout<<"EVERYBODY Get on the floor"<<std::endl;
				screwDrive(-.50);
			}
			
			//else if (fix this bitch later) basically just use right bumper as a toggle or something. 

			/* loop yield for a bit */
			sleepApp(20);
		}

		/* we've left the loop, likely due to gamepad disconnect */
		ldrive(0, 0);
		rdrive(0, 0);
		SDL_JoystickClose(joy);
		printf("gamepad disconnected\n");
	}

	SDL_Quit();
	return 0;
}
