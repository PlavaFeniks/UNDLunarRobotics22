#pragma once
#include <unistd.h>
#include <math.h>
#include <iostream>
//#include "movement.h"
//#include "infra/readSerial.h"
//#include "infra/SerialThread.h"


//TalonPair * Motor_RightF = new TalonPair(4);      //define wheel motors
//TalonPair * Motor_RightB = new TalonPair(2);
//TalonPair * Motor_LeftF = new TalonPair(3);
//TalonPair * Motor_LeftB = new TalonPair(1);


void adjust_angle(readSerial* ampSerial)
{
    std::cout << "start of angle" << std::endl;
    //ampSerial((char*)"/dev/ttyACM0");
    float *arr;
    arr = ampSerial->getSerialVals(10);
    float distR1 = arr[4];                  //get distance
    std::cout << "distR: " << distR1 <<std::endl;
    float distL1 = arr[5];
    std::cout << "distL: " << distL1 <<std::endl;
    int sepperation = 38;   //seperation between sensors. unit cm?
    float speed = 0;        //motor speed
    
    //line up straight to hopper
    while((distL1 > distR1 + 5) or (distR1 > distL1 + 5))
    {
        std::cout << arr[4] <<std::endl << arr[5] <<std::endl << arr[6] <<std::endl << arr[7] <<std::endl;
        if (distL1 > distR1 + 5)         //turn right
        {
            speed = 0.7;

			locomotion.SETSPEED(speed, speed);

            sleep(.5);
            
            arr = ampSerial->getSerialVals(10);
            distL1 = arr[4];            //update distance values
            distR1 = arr[5];
            //td::cout << "angle " << angl << std::endl;
            std::cout << "distL " << distL1 << std::endl;
            std::cout << "distR " << distR1 << std::endl << std::endl;
            //angl = asin((distL1 - distR1) / sepperation)* (180/3.14);       //update angle 

        }
    
        else if (distR1 > distL1 + 5)
        {
            std::cout << "loop\n";

            speed = -0.7;

            locomotion.SETSPEED(speed, speed);


            sleep(0.5);
            
            arr = ampSerial->getSerialVals(10);
            distL1 = arr[4];            //update distance values
            distR1 = arr[5];

            //std::cout << "angle " << angl << std::endl;
            std::cout << "distL " << distL1 << std::endl;
            std::cout << "distR " << distR1 << std::endl << std::endl;
            //angl = asin((distR1 - distL1) / sepperation)* (180/3.14);   //update angle
        }
        else {
            locomotion.SETSPEED(0, 0);
        }
    }
}


void adjust_dist(readSerial* ampSerial)
{
    //readSerial ampSerial((char*)"/dev/ttyACM0");
    float *arr;
    arr = ampSerial->getSerialVals(10);
    float distR1 = arr[5];
    float distL1 = arr[4];
    std::cout << "left: " << distL1 << std::endl << "right: " << distR1 << std::endl;

    int max_dist = 25;                              //distance to stop, account for offset of sensors

    float speed = 0.2;                              //speed to back up

    //back straight up to hopper until close enough
    std::cout <<"start backup" << std::endl;
    
    while ((distL1 > max_dist) || (distR1 > max_dist))
    {
        //backup
        
        locomotion.SETSPEED(speed, -speed);
        
        sleep(0.5);
    
        arr = ampSerial->getSerialVals(10);
        distR1 = arr[5];                                      //update distance
        distL1 = arr[4];

        std::cout << "left: " << distL1 << std::endl << "right: " << distR1 << std::endl;
    }

    std::cout <<"exit loop" << std::endl;
    locomotion.SETSPEED(0,0);
}

void depo_start(TalonPair * Motor_hopper_belt)
{
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    int min_load_cell = 0;      //value where hopper is considered empty
    float sleep_time = 1;         //amount of time between ld_cell_update
    int load_cel_val = 40;       //get the load cell value (prob not 40)
    int loop_count = 0;

    Motor_hopper_belt->SETSPEED(0.5);       //start hopper belt

    while (std::chrono::duration_cast<std::chrono::seconds>(end-begin).count() < 20 )
    {
		Motor_hopper_belt->SETSPEED(.5);
		end = std::chrono::steady_clock::now();
        loop_count++;
        //load_cel_val = new value;           //update ld cell value
       // sleep(sleep_time);                      //wait a little bit
       
    }

    Motor_hopper_belt->SETSPEED(0);             //stop hopper belt
}

void deposition(readSerial* ampSerial, TalonPair* Motor_hopper_belt)
{
    adjust_angle(ampSerial);
    std::cout << "angle adjusted\n Press Enter\n";
    adjust_dist(ampSerial);
    std::cout << "distance adjusted\n Press Enter\n";
    depo_start(Motor_hopper_belt);
    std::cout << "done\n";
}
