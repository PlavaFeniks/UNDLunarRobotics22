#include <iostream>
#include "../infra/readSerial.h"

using namespace std;

readSerial readSerialValues((char*)"/dev/ttyACM0");

int main(int argc, char * argv[]){
    float * values;
    
    values = readSerialValues.getSerialVals(10);
    for (int i = 0; i< 10; i++){
        cout<<values[i]<<endl;
    }

    
}