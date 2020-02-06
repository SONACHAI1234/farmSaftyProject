#include <termios.h>                                                         
#include <stdio.h>
#include <stdlib.h>	
#include <string.h>
#include <unistd.h>                                                          
#include <fcntl.h>                                                                                                               
#include <sys/types.h> 
#include <stdint.h>
#include <fcntl.h>
#include <sys/signal.h>
#include <sys/types.h>
#include <termios.h>
#include <time.h>
#include <stdbool.h>
#include <stropts.h>
#include <poll.h>	
//#include <pthread.h>
#include <errno.h>
#include <wiringPi.h>

#define BAUDRATE B9600                                                      
#define MODEMDEVICE "/dev/ttyUSB0"
#define LedPin 0

FILE * fp = NULL;

int set_interface_attribs (int fd, int speed, int parity)
{
	struct termios tty;
	memset (&tty, 0, sizeof tty);
	if (tcgetattr (fd, &tty) != 0)
	{
		printf ("error %d from tcgetattr", errno);
		return -1;
	}

	cfsetospeed (&tty, speed);
	cfsetispeed (&tty, speed);

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars

	tty.c_iflag &= ~IGNBRK;         // disable break processing
	tty.c_lflag = 0;                // no signaling chars, no echo,

	tty.c_oflag = 0;                // no remapping, no delays
	tty.c_cc[VMIN]  = 0;            // read doesn't block
	tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

	tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,

	tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
	tty.c_cflag |= parity;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;

	if (tcsetattr (fd, TCSANOW, &tty) != 0)
	{
		return -1;
	}
	return 0;
}

void set_blocking (int fd, int should_block)
{
	struct termios tty;
	memset (&tty, 0, sizeof tty);
	if (tcgetattr (fd, &tty) != 0)
	{
		printf("error %d from tggetattr", errno);
		return;
	}

	tty.c_cc[VMIN]  = should_block ? 1 : 0;
	tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

	if (tcsetattr (fd, TCSANOW, &tty) != 0)
		printf("error %d setting term attributes", errno);
}

void *myThreadFun(void *vargp) 
{ 
    char buffer[255];  
    printf("In thread\n");
    while(1)
    {
here:
	    memset(buffer, 0, sizeof(buffer));
	    fgets(buffer,255,fp);
	    if(strlen(buffer) == 0 )
		    goto here;
	    printf("buffer - %s\n",buffer);

    }
    return NULL; 
} 
int main(void)
{ 
	int fd;
	char commands[10][255] = {"\n\rAT+CMGF=1\r\n","\n\rAT+CMGR=1\r\n","\n\rAT+CPMS=\"SM\"\r\n","\n\rAT+CNMI=1,2,0,0,0\r\n"};
	//char buf[255];  
	int variable;
	struct pollfd fds[1];
	int ret, res;
	bool status = false;
//	pthread_t thread_id; 
//	pthread_create(&thread_id, NULL, myThreadFun, NULL); 
//	pthread_join(thread_id, NULL);
	/* open the device */
	if(wiringPiSetup() == -1) { //when initialize wiringPi failed, print message to screen
		printf("setup wiringPi failed !\n");
		return -1;
	}
	pinMode(LedPin, OUTPUT);
TRY_AGAIN:
	fd = open(MODEMDEVICE, O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (fd == 0)
	{
		perror(MODEMDEVICE);
		printf("Failed to open MODEMDEVICE \"/dev/ttyUSB0\"\n");
		goto TRY_AGAIN;
	}
	fp = fopen(MODEMDEVICE,"w+");

	set_interface_attribs (fd, BAUDRATE, 0);  // set speed to 19200 bps, 8n1 (no parity)
	set_blocking (fd, 0);                // set no blocking

	/* Open STREAMS device. */
	fds[0].fd = fd;
	fds[0].events = POLLRDNORM;
	char buffer[255] = {0};
	int count =0;
	int i=0;
	char buf[1024] = {0};
	sleep(10);
	for(i=0; i<4; i++)
	{
TRY_FPUT:
		if(fputs(commands[i],fp) == EOF)
			goto TRY_FPUT;
		sleep(0.4);
	}
	for(i=0; i<5; i++)
	{
		digitalWrite(LedPin, HIGH);
		usleep(1000*500);
		digitalWrite(LedPin, LOW);
		usleep(1000*500);
	}
#if 1
	fputs("\n\rAT+CMGF=1\r\n", fp);
	sleep(0.5);
	fputs("\n\rAT+CMGR=1\r\n", fp);
	sleep(0.5);
	fputs("\n\rAT+CPMS=\"SM\"\r\n", fp);
	sleep(0.5);
	fputs("\n\rAT+CNMI=1,2,0,0,0\r\n", fp);
	sleep(0.5);
#endif 
	for ( ; ; )		// forever
	{
	    fflush(fp);
	//wait for response
		memset(buffer, 0, sizeof(buffer));
here:
	    memset(buffer, 0, sizeof(buffer));
	    fgets(buffer,255,fp);
	    if(strstr(buffer,"***KISHANSTEEL***ON***KISHANSTEEL***"))
	    {
		    printf("Bhathi ON\n");
		    status = true;
		    digitalWrite(LedPin, HIGH);
	    	fputs("\n\rAT+CMGD=1\r\n", fp);
	    	sleep(0.2);
#if 0
		    count++;
		    printf("buffer - %s",buffer);
			strcat(buf,buffer);
			
#endif 
	    }
	    else if(strstr(buffer,"***KISHANSTEEL***OFF***KISHANSTEEL***"))
	    {
		status = false;
	    	printf("Bhathi OFF\n");
		digitalWrite(LedPin, LOW);
	    	fputs("\n\rAT+CMGD=1\r\n", fp);
	    	sleep(0.2);
	    }
	    else if(strstr(buffer,"OK"))
	    {
		printf(" ### OK ###\n");
	    }
	    else
	    {
#if 0
			strcat(buf,buffer);
			count = 0;
#endif 
    			goto here;
	    }
	    printf("GMS - %s\n",buffer);
	}
}
