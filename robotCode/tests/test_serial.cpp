#include <iostream>
#include "../infra/readSerial.h"
#include "../infra/logging.h"

using namespace std;

readSerial readSerialValues((char*)"/dev/ttyACM0");

int main(int argc, char * argv[]){
    pthread_t myThread;	
    thread_data * mydata = (thread_data*) malloc(sizeof(thread_data));
    string * entries = (string*) malloc(sizeof(string) * 4);
    TalonPair * mcs = (TalonPair*) malloc(sizeof(TalonPair) * 2);
    entries[0] = "bucket voltage";
    entries[1] = "bucket velocity";
    entries[2] = "screw voltage";
    entries[3] = "scew velocity";
    /*
    mcs[0] = buckets; 
    screwdriver = screwdriver; 
    */
    readSerial serialKiller((char*)"/dev/ttyACM0");
    mydata->serialConnection = &serialKiller;    

    mydata->dataEntries = entries;
    mydata->motorControllers = mcs;
    pthread_create(&myThread,NULL,logging,(void*)mydata);
    islogging = true;






    islogging = false;
    if(pthread_join(myThread,NULL) == 0){
        cout<<"Thread succesfully joined: shutting down";
    }
    free(mydata);

    return 0;



}