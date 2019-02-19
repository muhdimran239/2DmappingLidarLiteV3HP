#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>   /* File Control Definitions           */
#include <termios.h> /* POSIX Terminal Control Definitions */
#include <unistd.h>  /* UNIX Standard Definitions 	   */ 
#include <errno.h>   /* ERROR Number Definitions           */
#include <math.h>   
#include <string.h>


/*int pow(int x, int y){
	int res=1;
	for(int i=0;i<y;i++){
		res=res*x;
	}

	return res;
}*/


int main(void){
        int fd;/*File Descriptor*/
		
		printf("\n +----------------------------------+");
		printf("\n |        Serial Port Read          |");
		printf("\n +----------------------------------+");

		/*------------------------------- Opening the Serial Port -------------------------------*/

		/* Change /dev/ttyUSB0 to the one corresponding to your system */

        	fd = open("/dev/ttyUSB0",O_RDWR | O_NOCTTY);	/* ttyUSB0 is the FT232 based USB2SERIAL Converter   */
			   					/* O_RDWR   - Read/Write access to serial port       */
								/* O_NOCTTY - No terminal will control the process   */
								/* Open in blocking mode,read will wait              */
									
									                                        
									
        	if(fd == -1)						/* Error Checking */
            	   printf("\n  Error! in Opening ttyUSB0  ");
        	else
            	   printf("\n  ttyUSB0 Opened Successfully ");

	
		/*---------- Setting the Attributes of the serial port using termios structure --------- */
		
		struct termios SerialPortSettings;	/* Create theimplicit declaration of function structure                          */

		tcgetattr(fd, &SerialPortSettings);	/* Get the current attributes of the Serial port */

		/* Setting the Baud rate */
		cfsetispeed(&SerialPortSettings,B9600); /* Set Read  Speed as 9600                       */
		cfsetospeed(&SerialPortSettings,B9600); /* Set Write Speed as 9600                       */

		/* 8N1 Mode */
		SerialPortSettings.c_cflag &= ~PARENB;   /* Disables the Parity Enable bit(PARENB),So No Parity   */
		SerialPortSettings.c_cflag &= ~CSTOPB;   /* CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit */
		SerialPortSettings.c_cflag &= ~CSIZE;	 /* Clears the mask for setting the data size             */
		SerialPortSettings.c_cflag |=  CS8;      /* Set the data bits = 8                                 */
		SerialPortSettings.c_cflag &= ~CRTSCTS;       /* No Hardware flow Control                         */
		SerialPortSettings.c_cflag |= CREAD | CLOCAL; /* Enable receiver,Ignore Modem Control lines       */ 
		
		
		SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);          /* Disable XON/XOFF flow control both i/p and o/p */
		SerialPortSettings.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);  /* Non Cannonical mode                            */

		SerialPortSettings.c_oflag &= ~OPOST;/*No Output Processing*/
		
		/* Setting Time outs */
		SerialPortSettings.c_cc[VMIN] = 10; /* Read at least 10 characters */
		SerialPortSettings.c_cc[VTIME] = 0; /* Wait indefinetly   */


		if((tcsetattr(fd,TCSANOW,&SerialPortSettings)) != 0) /* Set the attributes to the termios structure*/
		    printf("\n  ERROR ! in Setting attributes");
		else
            printf("\n  BaudRate = 9600 \n  StopBits = 1 \n  Parity   = none");
			
	        /*------------------------------- Read data from serial port -----------------------------*/

		tcflush(fd, TCIFLUSH);   /* Discards old data in the rx buffer            */

		char read_buffer[4];   
		char int_buffer[6];/* Buffer to store the data received              */
		int  bytes_read = 0;    /* Number of bytes read by the read() system call */
 		int i = 0;
		int tempo;
		int distance; 
		int_buffer[5]='\0';
		while(1){
			read(fd,&read_buffer,1); /* Read the datq         */
			while(read_buffer[0]!='S')
			bytes_read = read(fd,&read_buffer,1);
			
			bytes_read = read(fd,&read_buffer,1);
			for(;read_buffer[0]!='\n';bytes_read = read(fd,&read_buffer,1)){
				printf("%c",read_buffer[0]);
				int_buffer[i]=read_buffer[0];
				i++;
			}//end for
			i=0;
				printf("\t int  distance = %d",atoi(int_buffer));
						   
			printf("\n +----------------------------------+\n\n\n");

		//tempo=100000;
		//while(tempo)
		//	tempo--;
        //usleep(500000);
		
		}//end while

		close(fd); /* Close the serial port */

    	}






///////gcc -Wall usb_code.c -o usb

