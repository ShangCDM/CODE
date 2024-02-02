#ifndef __USARTLGX_H
#define __USARTLGX_H

//Basic program for communication between STM32 and TX2 via USART

int openUart(void);//Open USART
int uartInit(int fd,int nSpeed, int nBits, char nEvent, int nStop);//initialization
void uartSend(int fd,char send_buf[], int length);//Send Data Function
int uartRead(int fd,char receive_buf[], int length);//Receive data function

#endif 
