#include<fcntl.h>  
#include<unistd.h>  
#include<termios.h> 
#include<stdio.h> 
#include<stdlib.h> 
#include<string.h> 
#include<errno.h> 
#include<poll.h> 
#include "ExperimentPkg/UsartTx2Basic.h"
#include "ExperimentPkg/Parameter.h"

int kk=0;
int openUart(void)
{
    int fd;
    fd = open("/dev/ttyTHS2", O_RDWR | O_NOCTTY | O_NDELAY);
    if(-1 == fd)
    {
        perror("Can't Open Serial Port");
        return (-1);
    }

    printf("fd-open=%d\n", fd);
    return fd;
}




/**
 * uartInit
 */
int uartInit(int fd,int nSpeed, int nBits, char nEvent, int nStop)
{
    struct termios newtio, oldtio;
    if  ( tcgetattr( fd, &oldtio)  !=  0) {
        perror("SetupSerial 1");
        printf("tcgetattr( fd,&oldtio) -> %d\n", tcgetattr( fd, &oldtio));
        return -1;
    }
    bzero( &newtio, sizeof( newtio ) );
    newtio.c_cflag  |=  CLOCAL | CREAD;
    newtio.c_cflag &= ~CSIZE;
    switch( nBits )
    {
    case 7:
        newtio.c_cflag |= CS7;
        break;
    case 8:
        newtio.c_cflag |= CS8;
        break;
    }
    switch( nEvent )
    {
    case 'o':
    case 'O': 
        newtio.c_cflag |= PARENB;
        newtio.c_cflag |= PARODD;
        newtio.c_iflag |= (INPCK | ISTRIP);
        break;
    case 'e':
    case 'E': 
        newtio.c_iflag |= (INPCK | ISTRIP);
        newtio.c_cflag |= PARENB;
        newtio.c_cflag &= ~PARODD;
        break;
    case 'n':
    case 'N':  
        newtio.c_cflag &= ~PARENB;
        break;
    default:
        break;
    }
    switch( nSpeed )
    {
    case 2400:
        cfsetispeed(&newtio, B2400);
        cfsetospeed(&newtio, B2400);
        break;
    case 4800:
        cfsetispeed(&newtio, B4800);
        cfsetospeed(&newtio, B4800);
        break;
    case 9600:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    case 115200:
        cfsetispeed(&newtio, B115200);
        cfsetospeed(&newtio, B115200);
        break;
    case 460800:
        cfsetispeed(&newtio, B460800);
        cfsetospeed(&newtio, B460800);
        break;
    default:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    }
    if( nStop == 1 )
        newtio.c_cflag &=  ~CSTOPB;
    else if ( nStop == 2 )
        newtio.c_cflag |=  CSTOPB;
    newtio.c_cc[VTIME]  = 0;
    newtio.c_cc[VMIN] = 0;
    tcflush(fd, TCIFLUSH);
    if((tcsetattr(fd, TCSANOW, &newtio)) != 0)
    {
        perror("com set error");
        return -1;
    }
    printf("set done!\n");
    return 0;
}


void uartSend(int fd,char send_buf[], int length)
{
    int w;
    w = write(fd, send_buf, length);
    if(w == -1)
    {
        printf("Send failed!\n");
    }
    else
    {
        //printf("Send success!\n");
    }
}

int uartRead(int fd,char receive_buf[], int length)
{
    int r;
    r = read(fd, receive_buf, length);
    kk++;
    /*for(int i = 0; i < r; i++)
    {
       printf("%c[%d]  ", (char)receive_buf[i],i);  
    }*/
   
    return r;

}