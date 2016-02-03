/*
 * Serial.cpp
 *
 *  Created on: Jan 13, 2014
 *      Author: raphael
 */

#include "Serial.h"
#include <stdio.h>      // standard input / output functions
#include <stdlib.h>
#include <string.h>     // string function definitions
#include <unistd.h>     // UNIX standard function definitions
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions
#include <iostream>

Serial::Serial() {
	setup();
}

Serial::~Serial() {
}

int Serial::setup()
{
	USB = open( "/dev/ttyUSB0", O_RDWR| O_NOCTTY );

	if ( USB < 0 )
	{
		cout << "Error " << errno << " opening " << "/dev/ttyUSB0" << ": " << strerror (errno) << endl;
	}

	struct termios tty;
	struct termios tty_old;
	memset (&tty, 0, sizeof tty);

	/* Error Handling */
	if ( tcgetattr ( USB, &tty ) != 0 )
	{
		cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << endl;
		return -1;
	}

	/* Save old tty parameters */
	tty_old = tty;

	/* Set Baud Rate */
	cfsetospeed (&tty, (speed_t)B115200);
	cfsetispeed (&tty, (speed_t)B115200);

	/* Setting other Port Stuff */
	tty.c_cflag     &=  ~PARENB;        // Make 8n1
	tty.c_cflag     &=  ~CSTOPB;
	tty.c_cflag     &=  ~CSIZE;
	tty.c_cflag     |=  CS8;

	tty.c_cflag     &=  ~CRTSCTS;       // no flow control
	tty.c_cc[VMIN]      =   1;                  // read doesn't block
	tty.c_cc[VTIME]     =   5;                  // 0.5 seconds read timeout
	tty.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines

	/* Make raw */
	cfmakeraw(&tty);

	/* Flush Port, then applies attributes */
	tcflush( USB, TCIFLUSH );
	if ( tcsetattr ( USB, TCSANOW, &tty ) != 0)
	{
		cout << "Error " << errno << " from tcsetattr" << endl;
	}
	return 0;
}

void Serial::send(list<char> buffer, char flag)
{
	if (USB < 0)
	{
		if (setup() == -1)
			return;
	}

	for (list<char>::iterator it = buffer.begin(); it != buffer.end(); it++)
	{
		if (*it == 0x00 || *it == 0x01 || *it == 0x02 || *it == 0x7D)
		{
			buffer.insert(it, 0x7D);
		}
	}
	buffer.insert(buffer.begin(), flag);
	buffer.push_back(flag);
	char *c = new char[buffer.size()];
	copy(buffer.begin(), buffer.end(), c);
	write(USB, c, buffer.size());
}
