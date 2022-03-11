#include <iostream>
#include "../infra/readSerial.h"
#include <pthread.h>

float floats[10];

using namespace std;



int main(int argc, char * argv[]){
    float * values;
    readSerial readSerialValues((char*)"/dev/ttyACM0");
        while(true){
            values = readSerialValues.getSerialVals(10);

            for (int i = 0; i<10; i++){
                
            cout<<values[i]<<endl;
            }
            sleep(1);
        }

    return 0;
 }

