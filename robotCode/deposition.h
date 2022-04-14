#pragma once
#include <unistd.h>
#include <math.h>
#include <iostream>
//#include "movement.h"
//#include "infra/readSerial.h"
//#include "infra/SerialThread.h"


TalonPair * Motor_RightF = new TalonPair(4);      //define wheel motors
TalonPair * Motor_RightB = new TalonPair(2);
TalonPair * Motor_LeftF = new TalonPair(3);
TalonPair * Motor_LeftB = new TalonPair(1);
TalonPair * Motor_hopper_belt = new TalonPair(7);

void adjust_angle(readSerial* ampSerial)
{
    std::cout << "start of angle" << std::endl;
    //ampSerial((char*)"/dev/ttyACM0");
    float *arr;
    arr = ampSerial->getSerialVals(10);
    float distR1 = arr[7];                  //get distance
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
            std::cout << "loop\n";
            speed = 0.7;

            Motor_RightF->SETSPEED(speed);                        //set the turn speed
            Motor_RightB->SETSPEED(speed);
            Motor_LeftF->SETSPEED(speed);
            Motor_LeftB->SETSPEED(speed);


            sleep(.5);
            
            arr = ampSerial->getSerialVals(10);
            distL1 = arr[5];            //update distance values
            distR1 = arr[7];
            //td::cout << "angle " << angl << std::endl;
            std::cout << "distL " << distL1 << std::endl;
            std::cout << "distR " << distR1 << std::endl << std::endl;
            //angl = asin((distL1 - distR1) / sepperation)* (180/3.14);       //update angle 

        }
    
        else if (distR1 > distL1 + 5)
        {
            std::cout << "loop\n";

            speed = -0.7;

            Motor_RightF->SETSPEED(speed);                        //set the turn speed
            Motor_RightB->SETSPEED(speed);
            Motor_LeftF->SETSPEED(speed);
            Motor_LeftB->SETSPEED(speed);


            sleep(0.5);
            
            arr = ampSerial->getSerialVals(10);
            distL1 = arr[5];            //update distance values
            distR1 = arr[7];

            //std::cout << "angle " << angl << std::endl;
            std::cout << "distL " << distL1 << std::endl;
            std::cout << "distR " << distR1 << std::endl << std::endl;
            //angl = asin((distR1 - distL1) / sepperation)* (180/3.14);   //update angle
        }
        else {
            Motor_RightF->SETSPEED(0);                        //stop the robot
            Motor_RightB->SETSPEED(0);
            Motor_LeftF->SETSPEED(0);
            Motor_LeftB->SETSPEED(0);
        }
    }
}


void adjust_dist(readSerial* ampSerial)
{
    //readSerial ampSerial((char*)"/dev/ttyACM0");
    float *arr;
    arr = ampSerial->getSerialVals(10);
    float distR1 = arr[7];
    float distL1 = arr[5];
    std::cout << "left: " << distL1 << std::endl << "right: " << distR1 << std::endl;

    int max_dist = 25;                              //distance to stop, account for offset of sensors

    float speed = 0.2;                              //speed to back up

    //back straight up to hopper until close enough
    std::cout <<"start backup" << std::endl;
    
    while ((distL1 > max_dist) || (distR1 > max_dist))
    {
        

        //backup
        
        Motor_RightF->SETSPEED(-1 * speed);                        //set the backup speed
        Motor_RightB->SETSPEED(-1 * speed);
        Motor_LeftF->SETSPEED(speed);
        Motor_LeftB->SETSPEED(speed);
        
        sleep(0.5);
        /*
        Motor_RightF->SETSPEED(0);                        //stop motors when done
        Motor_RightB->SETSPEED(0);
        Motor_LeftF->SETSPEED(0);
        Motor_LeftB->SETSPEED(0);
        */
    
        arr = ampSerial->getSerialVals(10);
        distR1 = arr[7];                                      //update distance
        distL1 = arr[5];

        std::cout << "left: " << distL1 << std::endl << "right: " << distR1 << std::endl;
    }

    std::cout <<"exit loop" << std::endl;
    Motor_RightF->SETSPEED(0);                        //stop motors when done
    Motor_RightB->SETSPEED(0);
    Motor_LeftF->SETSPEED(0);
    Motor_LeftB->SETSPEED(0);
}

void depo_start()
{
    int min_load_cell = 0;      //value where hopper is considered empty
    float sleep_time = 0.5;         //amount of time between ld_cell_update
    int load_cel_val = 40;       //get the load cell value (prob not 40)
    int loop_count = 0;

    Motor_hopper_belt->SETSPEED(0.5);       //start hopper belt

    while (load_cel_val >= min_load_cell)
    {
        loop_count++;
        //load_cel_val = new value;           //update ld cell value
        sleep(sleep_time);                      //wait a little bit
        if (loop_count >= 120)
        {
            break;
        }
    }

    Motor_hopper_belt->SETSPEED(0);             //stop hopper belt
}

void deposition()
{
    adjust_angle(new readSerial((char*)"/dev/tty/ACM0"));
    std::cout << "angle adjusted\n Press Enter\n";
    adjust_dist(new readSerial((char*)"/dev/tty/ACM0"));
    std::cout << "distance adjusted\n Press Enter\n";
    depo_start();
    std::cout << "done\n";
}
