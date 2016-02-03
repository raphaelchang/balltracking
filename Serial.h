/*
 * Serial.h
 *
 *  Created on: Jan 13, 2014
 *      Author: raphael
 */

#ifndef SERIAL_H_
#define SERIAL_H_

#include <list>

using namespace std;

class Serial {
public:
	Serial();
	~Serial();

	void send(list<char> buffer, char flag);

private:
	int setup();

	int USB;
};

#endif /* SERIAL_H_ */
