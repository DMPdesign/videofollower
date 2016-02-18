/*
 * servo.cpp
 *
 *  Created on: 28/nov/2015
 *      Author: Dazzi Martino,
 *              Mele Leandro Julian,
 *      	Pilotto Alessandro
 *
 *      Università degli Studi di Udine,
 *      Via delle Scienze 208,
 *      Udine, Italy
 */

#include <stdio.h>
#include <stdlib.h>
#include "SerialStream.h"

#include "main.hpp"
#include "servo.hpp"

using namespace std;
using namespace LibSerial;

SerialStream servo;

char connectServo (char serialName[])
{
    	char error = ERROR_NONE;

    	servo.Open(serialName);
    	servo.SetBaudRate(SerialStreamBuf::BAUD_9600);
    	servo.SetCharSize(SerialStreamBuf::CHAR_SIZE_8);
    	servo.SetParity(SerialStreamBuf::PARITY_NONE);
    	servo.SetNumOfStopBits(1);

    	if (!servo.IsOpen())
    		error = ERROR_SERVO;

    return error;
}

void disconnectServo ()
{
	servoSet(SPEED_5);
	servoSet(TO_DEFAULT);
	servo.Close();

	return;
}

void servoSet (char param)
{
	char str[2];

	str[0] = param;
	str[1] = '\n';

	servo << str;

	return;
}

char servoGetParam (char code)
{
	char param = 0;
	char str[2];

	str[0] = code;
	str[1] = '\n';

	servo << str;

	servo.get(param);

	return param;
}
