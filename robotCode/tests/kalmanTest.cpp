#include "../infra/kalmanFilter.h"
#include <stdlib.h>
#include <iostream>

using namespace std;

int main(int argc, char *argv[]){
    float * initVals = (float *)malloc(sizeof(float) * 3);
    initVals[XH] = 3;
    initVals[XHD] = 4.86;
    initVals[P] = .01;
    for( int i = 0; i < 3; i++) cout<<initVals[i]<<endl;
    initVals = kalmanFilter(initVals, 4.86, 3.70161, 3068.260964);
    for(int i = 0; i < 3; i++)
        cout<<initVals[i]<<endl;
    return 0;


}