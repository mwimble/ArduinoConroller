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

#include <iostream>
using namespace std;

int main(int argc, char** argv) {
  char* errnoString;
  size_t size = 1024;
  errnoString = (char*)malloc(size);

  printf("**** t1 test program ****\n");

  int fd;                          // File descrition
  char fileName[] = "/dev/i2c-1";  // Name of the port we will be using
  int address = 0x0C;              // Address of CMPS03 shifted right one bit
  unsigned char buf[32];  // Buffer for data being read/ written on the i2c bus

  if ((fd = open(fileName, O_RDWR)) < 0) {  // Open port for reading and writing
    printf("Failed to open i2c port\n");
    exit(1);
  }

  if (ioctl(fd, I2C_SLAVE, address) < 0) {  // Set the port options and set the
                                            // address of the device we wish to
                                            // speak to
    printf("Unable to get bus access to talk to slave\n");
    exit(1);
  }

  //buf[0] = 1;  // This is the register we want to read from
  cout << "Enter character: ";
  cin >> buf[0];

  if ((write(fd, buf, 1)) != 1) {  // Send register we want to read from
    printf("Error writing to i2c slave: %d\n", errno);
    exit(1);
  }

  delay(100);

  printf("Result >>");
  do {
    int rdLen = read(fd, &buf[0], 1);
    if (rdLen != 1) {
      if (rdLen < 0) {
        strerror_r(errno, errnoString, size);
        printf("Unable to read from slave, read result: %d, errno: %d, %s\n",
               rdLen, errno, errnoString);
        exit(1);
      } else {
        printf("Bad rdlen: %d ", rdLen);
      }
    } else {
      printf("%c", buf[0]);
    }
  } while (buf[0] != '\n');

  printf("<<<\n");

  return 0;
}
