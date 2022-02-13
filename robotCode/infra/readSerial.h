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
};

readSerial::readSerial(char* abosolutePath)
{   
    fd = open(abosolutePath, O_RDONLY|O_NOCTTY); //O_NONBLOCK
    if (fd < 0) {
    printf("Error %i from open: %s\n", errno, strerror(errno));
    return;
    }
    struct termios tty;
    char read_buf;


    cfsetspeed(&tty,B9600);
    if(tcgetattr(fd, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    }   
    
    cfmakeraw(&tty);
    
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
    while(read_buf != ','){
        outPutString += read_buf;
        n =  read(fd, &read_buf,1);
    } 

    return(outPutString);

    

}

readSerial::~readSerial()
{
    close(fd);
}
