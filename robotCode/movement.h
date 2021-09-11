#include <iostream>
#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"

using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;




void  setMotors (TalonSRX Motor, double Percentage)
{
	Motor.Set(ControlMode::PercentOutput, Percentage);

}

