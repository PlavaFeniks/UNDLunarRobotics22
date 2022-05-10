#include <stdio.h>
#include <stdlib.h>


#define w2 0  //constant velocity guess
#define kb 1137.03 // motor constant 
#define Res .22641 //armature resistance
#define Irest .429756 //base current draw
#define q 0.001 //pr
#define r .05


enum cVals {XH, XHD, P};

float * kalmanFilter(float * vals, float current, float vb, float w1){ 
    float k;
    float xh;
    float xhd;
    float p;

    k = vals[P]/(vals[P]+ r);
    xh = vals[XH]+ k*(current -vals[XH]);
    xh = xh+ vals[XHD]*.006;

    //predictor
    xhd = (((vb - w1/ kb)/Res + (-1*(w2/kb))/Res+Irest) - xh)/2;
    
    p = (1 - k)* vals[P];
    p = p + q;
    free(vals);

    float * kVals = (float *)malloc(sizeof(float) * 3);
    kVals[XH] = xh;
    kVals[XHD] = xhd;
    kVals[P] = p;

    return(kVals);


    
}