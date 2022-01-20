#define Phoenix_No_WPI // remove WPI dependencies
#include <iostream>
#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"

using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;
enum MotorControl {PERCENT, PID};

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
	void SETSPEED(float speed){

		switch (motorType){
			case PERCENT:
				if(speed > 1 or speed > limit[1]){
					mc ->Set(ControlMode::PercentOutput, limit[1]);
				}
				else if ( speed < 0 or speed < limit[0]){
					mc ->Set(ControlMode::PercentOutput, limit[1]);

				}
				else{

					float targetVal = .23;
					float rat = this->READSENSORS()/ targetVal;
					float endResult = .04;
					/*
					do arithmetic
					*/
					mc->Set(ControlMode::PercentOutput, endResult);
					

				}
			case PID:
				std::cout<<"nerd";

			
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


