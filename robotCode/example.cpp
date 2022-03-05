//Input/output stuff
#include <string>
#include <iostream>
#include <stdio.h>
#include <chrono>
//thread stuff
#include <thread>
#include <pthread.h>
#include <SDL2/SDL.h> //Read joystick events
//file system stuff
#include <filesystem>
#include <dirent.h>
#include <fcntl.h>
#include <fstream>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <termios.h>
#include <errno.h>
#include "chassis.h" //custom headers

using namespace std;

#define locoThresh .1
#define upperDlim .7
#define lowerDlim .0
const float screwSpeed = 0.4;
//define buttons and joysticks here
enum Buttons{XBUTTON};
enum Joystics{LEFTSTICK};
//Declare Prototypes please
void sleepApp(int ms);
void fullStop();
void setup();
//Setup MotorController devices
chassis Locomotion(true);
TalonPair screw(5);
TalonPair buckets(6);
TalonPair hopper(7);

//global vars
double diggerDSpeed;
bool logging = false;

void setup(){
	//Runs once, sets TalonPairs to manual
	screw.SWITCHMANUAL();
	buckets.SWITCHMANUAL();
	hopper.SWITCHMANUAL();
}

void fullStop(){
	//stops all motorcontrollers
	double stop = 0.0;
	Locomotion.SETSPEED(stop,stop);
	screw.SETSPEED(stop);
	buckets.SETSPEED(stop);
	hopper.SETSPEED(stop);
}
//move to deposition headerfile
void stepDigger(double stepFunc){
	//increases global digging speed
	double tSpeed = 0;
	tSpeed = diggerDSpeed + stepFunc;
	if ((tSpeed >= upperDlim) or (tSpeed<=lowerDlim)){
		std::cout<<"THIS IS AN INVALID NUMBER CANNOT GO ABOVE" <<upperDlim<< "or BELOW "<<lowerDlim<<std::endl;
		return;
	}
	else{
		diggerDSpeed = tSpeed;
		std::cout<<"DIGGER SPEED PERCENT CHANGED: "<<diggerDSpeed<<std::endl;
		sleepApp(500);
		return;
		
	}
}
void sleepApp(int ms){
	//sleep but in milleseconds
	this_thread::sleep_for(chrono::milliseconds(ms));
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

int main() {
	pthread_t myThread;	
	setup();

	std::string interface;
	interface = "can0";
	int temp; 
	if (temp = (ctre::phoenix::platform::can::SetCANInterface(interface.c_str())) == -1){
		perror("");
		std::_Exit(0);
	}
	
	// Comment out the call if you would rather use the automatically running diag-server, note this requires uninstalling diagnostics from Tuner. 
	// c_SetPhoenixDiagnosticsStartTime(-1); // disable diag server, instead we will use the diag server stand alone application that Tuner installs

	/* setup drive */	

	while (true) {
		
		fullStop();
		diggerDSpeed = .25;
		// wait for gamepad
		printf("Waiting for gamepad...\n");
		while (true) {
			SDL_Quit();
            SDL_SetHint(SDL_HINT_NO_SIGNAL_HANDLERS, "1"); //so Ctrl-C still works
			SDL_Init(SDL_INIT_JOYSTICK);

			/* poll for gamepad */
			int res = SDL_NumJoysticks();
			if (res < 0) { printf("Err = %i\n", res); }
			else break;

			sleepApp(20);
		}
		printf("Gamepad found\n");

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
			//always default to stopping motors
			fullStop();
			if (SDL_PollEvent(&event)) {
				if (event.type == SDL_QUIT) { break; }
				if (event.jdevice.type == SDL_JOYDEVICEREMOVED) { break; }
			}
			/* grab some stick values */
			double y = ((double)SDL_JoystickGetAxis(joy, 0)) / -32767.0;
			double turn = ((double)SDL_JoystickGetAxis(joy, 1)) / -32767.0;
			double ry = ((double)SDL_JoystickGetAxis(joy, 3)) / -32767.0;
			double rturn = ((double)SDL_JoystickGetAxis(joy, 4)) / -32767.0;

			for (int i = 0; i< num_buttons; i++){
				if (SDL_JoystickGetButton(joy,i)){
					cout<<"This is button " << i << "\n";
				}
			}
			/* [SAFETY] only enable drive if top left shoulder button is held down */
			if (SDL_JoystickGetButton(joy, 4)) {
				ctre::phoenix::unmanaged::FeedEnable(100);
			}

			if(abs(turn) > locoThresh or abs(rturn) > locoThresh){
				Locomotion.SETSPEED(y-turn, ry-rturn);
			}

			if ((double)SDL_JoystickGetAxis(joy,2) > 0){
				buckets.SETSPEED(diggerDSpeed);
			}

			if ((double)SDL_JoystickGetAxis(joy,5) > 0){
				hopper.SETSPEED(.50);
			}

			else if (SDL_JoystickGetButton(joy, 5)) {
				hopper.INVERT();
				buckets.INVERT();
				sleepApp(500);
			}
			//Set up/stop logging
			if (SDL_JoystickGetButton(joy,2)){
				if (!logging){
				logging = true;
				//pthread_create(&myThread,NULL,loggingThread2,NULL);
				sleep(2);
				}
			}
			else if(SDL_JoystickGetButton(joy,3)){
				if(logging){
				logging = false;
				cout<<"Requesting Logging Thread to stop. Please wait\n";
				if(pthread_join(myThread, NULL) == 0)
				cout<<"Thread has successfully been joined, no longer a zombie\n";
				sleep(2);
				}
				else{
					cout<<"Oops. Something went wrong\n";
					perror("");
				}
			}
		
			if (SDL_JoystickGetHat(joy,0) == SDL_HAT_UP){
				stepDigger(.05);
			}
			else if (SDL_JoystickGetHat(joy,0) == SDL_HAT_DOWN){
				stepDigger(-.05);
			}
			else if (SDL_JoystickGetHat(joy,0) == SDL_HAT_LEFT){
				screw.SETSPEED(screwSpeed);
			}
			else if (SDL_JoystickGetHat(joy,0) == SDL_HAT_RIGHT){
				screw.SETSPEED(-(screwSpeed));
			}
			/* loop yield for a bit */
			sleepApp(20);
		}

		/* we've left the loop, likely due to gamepad disconnect */
		fullStop();
		SDL_JoystickClose(joy);
		printf("gamepad disconnected\n");
	}

	SDL_Quit();
	return 0;
}
