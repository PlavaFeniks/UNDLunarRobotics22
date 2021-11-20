#include <iostream>
#include <fstream>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>

char aboslute_path[] = "/dev/ttyACM0";


using namespace std;
float *readSerial(int fd){
    char read_buf[256];
    int n = read(fd, &read_buf, sizeof(read_buf));
    /*
    while(true)
    {
        int n = read(fd, &read_buf, sizeof(read_buf));

        cout<<read_buf;
        sleep(1);
    }
    */
    
   float *returnVals = (float*)malloc(sizeof(float) *3);
   int count = 0;
   string tempString;
   for(int i = 0; i<sizeof(read_buf); i++){
       if (read_buf[i] = ',' or read_buf[1] == '\0'){
           returnVals[count] = stof(tempString, NULL);
           tempString == "";
           count++;
           if (read_buf[1] == '\0')break;
       }
       else{
           tempString += read_buf[i];
       }
   }

    return(returnVals);
    
}

int main(int argc, char argv[]){
    
    int serial_fd;
    serial_fd = open(aboslute_path, O_RDONLY );
    if (serial_fd < 0) {
    printf("Error %i from open: %s\n", errno, strerror(errno));
    }
    if (argc > 1){
        float *term = readSerial(serial_fd);

        for(int i = 0; i<3 ; i++){
            term[1];
        }
        

    }
    else{

    
        struct termios tty;



        cfsetspeed(&tty,B9600);
        if(tcgetattr(serial_fd, &tty) != 0) {
            printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        }   
        char read_buf[256];
        int n = read(serial_fd, &read_buf, sizeof(read_buf));
        double current = 0.0;
        while(true)
        {
            int n = read(serial_fd, &read_buf, sizeof(read_buf));

            current = atof(read_buf);

            sleep(1);
        }
    }

    close(serial_fd);
    return(0);
}