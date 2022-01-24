#define Phoenix_No_WPI // remove WPI dependencies
#include <iostream>
#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"

using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;
enum MotorControl {PERCENT, POSITION, VELOCITY};

class TalonPair{
	public:
		TalonSRX *mc;
		SensorCollection *sc;
		float limit[2];
		int motorType;
	
	TalonPair(int motor){
		mc = new TalonSRX(motor);
		sc = &(mc)->GetSensorCollection();
		limit[0] = 0.0;
		limit[1] = 1.0;
		motorType = PERCENT;
	}
	TalonPair(int, int, float *);
	TalonPair(int, int);
	void SETSPEED(float speed){

		switch (motorType){
			case PERCENT:
				if(abs(speed) > limit[1]){
					mc ->Set(ControlMode::PercentOutput, limit[1]);
				}
				else if ( abs(speed) < limit[0]){
					mc ->Set(ControlMode::PercentOutput, limit[1]);

				}
				else{
					mc->Set(ControlMode::PercentOutput, speed);

				}
			case VELOCITY:
				/*include PID math HERE*/
				std::cout<<"nerd";
			case POSITION:
				std::cout<<"HOWDY";
				/*include velocity math here*/

			
		}

	}
	int READSENSORS(){
		try
		{
			return(sc->GetQuadratureVelocity());
		}
		catch(const std::exception& e)
		{
			std::cerr << e.what() << '\n';
			return(0);
		}
		
	}

};

TalonPair::TalonPair(int motor, int ControlMode){
	mc = new TalonSRX(motor);
		sc = &(mc)->GetSensorCollection();
		limit[0] = 0.0;
		limit[1] = 1.0;
		motorType = ControlMode;
};

TalonPair::TalonPair(int motor, int ControlMode, float *limits){
	mc = new TalonSRX(motor);
		sc = &(mc)->GetSensorCollection();
		
		(abs(limits[0]) < 0)? limit[0] = 0: limit[0] = limits[0];
		(abs(limits[1]) > 1)? limit[1] = 1: limit[1] = limits[1];
		motorType = ControlMode;
};
