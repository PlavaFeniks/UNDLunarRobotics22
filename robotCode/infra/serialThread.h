#include "readSerial.h"
#include <pthread.h>
#define ard_elements 10

pthread_mutex_t readLock;
pthread_t ard_thread;
float * ard_values;


//Call this to have thread in the background
void setup_ard_Thread(readSerial* serialConnection){
    pthread_mutex_init(&readLock,0);


    pthread_create(&ard_thread, NULL, &updateArd_values, (void*)serialConnection);
}

void * updateArd_values(void* threadData){
    
    
    readSerial * serialConnection = (readSerial *)threadData;

    //Always run in the background. This was simpler than implementing a flag-checking schema
    while true{
        pthread_mutex_lock(&readLock);
        ard_values = serialConnection->getSerialVals(ard_elements);
        pthread_mutex_unlock(&readLock);
    }

}

//returns pointer to float array 
float * readArd_values(){

    float * ard_vals = (float*)malloc(sizeof(float) * ard_elements);
    pthread_mutex_lock(&readLock);
    ard_vals = ard_values;
    pthread_mutex_unlock(&readLock);
    return(ard_vals);
}
