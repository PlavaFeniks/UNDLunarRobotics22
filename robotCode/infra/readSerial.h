#include <iostream>
#include <fcntl.h>
#include <stdlib.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <error.h>
#include <string.h>
#include <unistd.h>

using namespace std;


class readSerial
{
private:
    int fd;
public:
    readSerial(char*);
    ~readSerial();
    string getSerial();
    float *getSerialVals(int);
};

readSerial::readSerial(char* abosolutePath)
{   
    fd = open(abosolutePath, O_RDONLY|O_NOCTTY); //O_NONBLOCK | O_RDONLY
    if (fd < 0) {
    printf("Error %i from open: %s\n", errno, strerror(errno));
    return;
    }
    struct termios tty;
    char read_buf;


    
    if(tcgetattr(fd, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    }   
    
    cfmakeraw(&tty);

    cfsetspeed(&tty,B115200);
    tty.c_cflag |= CS8;
    tty.c_lflag |=(CLOCAL| CREAD);
    tty.c_iflag &= ~(IXOFF|IXON);
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 0;
    
    
    
    tcsetattr(fd,TCSANOW,&tty);
    

}

string readSerial::getSerial(){
    char read_buf;
    string outPutString;
    int n = read(fd, &read_buf, 1);
    while(read_buf != ',' and read_buf != ';'){
        outPutString += read_buf;
        n =  read(fd, &read_buf,1);
    } 
    if (read_buf == ';') {
    outPutString += read_buf;
    cout<<"semicolon Found"<<endl;
    }
    return(outPutString);

    

}
float * readSerial::getSerialVals(int value_count){
    float * float_vals = (float*)(malloc(sizeof(float) * value_count));
    int count = 0;
    for(int i = 0; i < value_count; i++){
        string tempString = getSerial();
        if(tempString[tempString.length() - 1] == ';' || tempString[0]==';'){
            count++;
            if (count != value_count){
                return(getSerialVals(value_count));
            }
            else{
                tempString = tempString.substr(0, tempString.length()- 1);
                float_vals[i] = stof(tempString);
            }
        }
        else{
        float_vals[i] = stof(tempString);
        }
        count ++;
    }
    
    return(float_vals);

}

readSerial::~readSerial()
{
    close(fd);
}
