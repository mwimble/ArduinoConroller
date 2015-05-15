#include <cerrno>
#include <stdio.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include <wiringPi.h>


int main(int argc, char **argv)
{
	printf("**** t1 test program ****\n");
	
	int fd;														// File descrition
	char fileName[] = "/dev/i2c-1";								// Name of the port we will be using
	int  address = 0x0C;										// Address of CMPS03 shifted right one bit
	unsigned char buf[32];										// Buffer for data being read/ written on the i2c bus
	
	if ((fd = open(fileName, O_RDWR)) < 0) {					// Open port for reading and writing
		printf("Failed to open i2c port\n");
		exit(1);
	}
	
	if (ioctl(fd, I2C_SLAVE, address) < 0) {					// Set the port options and set the address of the device we wish to speak to
		printf("Unable to get bus access to talk to slave\n");
		exit(1);
	}
		
	buf[0] = 1;													// This is the register we want to read from
	
	if ((write(fd, buf, 1)) != 1) {								// Send register we want to read from	
		printf("Error writing to i2c slave: %d\n", errno);
		exit(1);
	}
	
	delay(50);
	
	printf("Result >>");
	do {
	    if (read(fd, buf, 1) != 1) {
    		printf("Unable to read from slave\n");
	    	exit(1);
    	} else {
	        printf("%c", buf[0]);
        }
	} while (buf[0] != '\n');
	printf("<<<\n");
// 	int rlen = read(fd, &buf[0], 12);
// 	if (rlen != 12) {								// Read back data into buf[]
// 		printf("Unable to read from slave\n");
// 		exit(1);
// 	} else {
// 	    buf[12] = 0;
// 	    printf("Result [len=%d] >>", rlen);
// 	    for (int i = 0; i < rlen; i++) {
// 	        printf("%c", buf[i]);
//         }
//         
//         printf("<<\n");
//         
        //     unsigned char highByte = buf[2];
//             unsigned char lowByte = buf[3];
//             unsigned int result = (highByte <<8) + lowByte;			// Calculate bearing as a word value
//             printf("Software v: %u \n",buf[0]);
//             printf("Bearing as byte: %u \n",buf[1]);
//             printf("Bearing as decimal: %u.%u\n",result/10, result%10); // display bearing with decimal place
//	}
	
	return 0;
}
