#pragma once
#include "movement.h"

class chassis
{
    private:
        TalonPair * __bleft;
        TalonPair * __brght;
        TalonPair * __fleft;
        TalonPair * __frght;
    public:
        chassis(bool);
        chassis(float *);
        chassis(TalonPair*, TalonPair* , TalonPair*, TalonPair*);
        float * get_quadrature_velocities();
        float * get_voltages();
        void setup(bool);
        void SETSPEED(double, double);
        void PIVOT(int, double);
        void SWITCHMANUAL();

};

chassis::chassis(float* PID_val){
    float limits[2] = {0,1};
    
    float *PID_vals= (float*)malloc(sizeof(float)*4);

    PID_vals[PID_P] = .4;          // should probably tune one for buckets and one for screw
	PID_vals[PID_I] = .000;
	PID_vals[PID_D] = .8;
	PID_vals[PID_F] = .32;
    
    
    
    __bleft = new TalonPair(RearLeft, VELOCITY, limits, PID_vals);
    __brght = new TalonPair(RearRight, VELOCITY, limits, PID_vals);
    __frght = new TalonPair(FrontRight, VELOCITY, limits, PID_vals);
    __fleft = new TalonPair(FrontLeft, VELOCITY, limits, PID_vals);
    __fleft->INVERT();
    __brght->INVERT();
    setup(false);
    cout<<"Successfully created chassis control with velocitySettings"<<endl;
}

chassis::chassis(bool isManual){
    __bleft = new TalonPair(RearLeft);
    __brght = new TalonPair(RearRight);
    __frght = new TalonPair(FrontRight);
    __fleft = new TalonPair(FrontLeft);
    setup(isManual);
    cout<<"Successfully created chassis control group"<<endl;
};
chassis::chassis(TalonPair* b_left, TalonPair* b_rght , TalonPair* f_left, TalonPair* f_rght)
{
    __bleft = b_left;
    __brght = b_rght;
    __fleft = f_left;
    __frght = f_rght;

    setup(false);

};
void chassis::SETSPEED(double _lSpeed, double _rspeed){
    __fleft->SETSPEED(_lSpeed);
    __frght->SETSPEED(_rspeed);
    __bleft->SETSPEED(_lSpeed);
    __brght->SETSPEED(_rspeed);
    cout<< "Chassis Speed Left: " <<_lSpeed << " Right: " <<_rspeed<<endl;
}

//Use int for issues regarding likely precision
//More than likely just use like time or something, or rather a boolean sensor input
void chassis::PIVOT(int angle, double speed){
    //catch invalid inputs greater than 360 and convert to valid input
    //In case angle is greater than 180, faster to turn counter clockwise instead of clockwise
    if(abs(angle % 360) > 180){
        speed = -speed;
        cout<<"Pivoting counter-clockwise at speed " <<speed <<endl;
    }else{
        cout<<"Pivoting clockwise at speed " <<speed <<endl;
    };
    
    __fleft->SETSPEED(speed);
    __frght->SETSPEED(-speed);
    __bleft->SETSPEED(speed);
    __brght->SETSPEED(-speed);

    return;
    
}
//This sets up manual and inverts one side to matchrotations, __brght should actually be inverted if you want controller to be nice
void chassis::setup(bool isManual){
    //__brght->INVERT();
    //__frght->INVERT();
    
    __bleft->INVERT();
    __fleft->INVERT();
    if(isManual){
        __brght->SWITCHMANUAL();
        __frght->SWITCHMANUAL();
        __bleft->SWITCHMANUAL();
        __frght->SWITCHMANUAL();
    }
    return;
}

//Get Quad Velocities from motor controllers
float * chassis::get_quadrature_velocities(){

    float *quadrature_velocities = (float*)malloc(sizeof(float*) * 4);
    
    quadrature_velocities[0] = __bleft->getQuadVelocity();
    quadrature_velocities[1] = __brght->getQuadVelocity();
    quadrature_velocities[2] = __fleft->getQuadVelocity();
    quadrature_velocities[3] = __frght->getQuadVelocity();

    return(quadrature_velocities);

}

//Get Voltages from motor controllers. Leverages chassis::getVoltage
float * chassis::get_voltages(){
    float *voltages = (float*)malloc(sizeof(float*) * 4);
    
    voltages[0] = __bleft->getVoltage();
    voltages[1] = __brght->getVoltage();
    voltages[2] = __fleft->getVoltage();
    voltages[3] = __frght->getVoltage();

    return(voltages);
}

