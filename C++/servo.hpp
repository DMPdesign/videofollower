/*
 * servo.hpp
 *
 *  Created on: 28/nov/2015
 *      Author: Dazzi Martino,
 *      		Mele Leandro Julian,
 *      		Pilotto Alessandro
 *
 *              Università degli Studi di Udine,
 *              Via delle Scienze 208,
 *              Udine, Italy
 */

#ifndef SRC_SERVO_HPP_
#define SRC_SERVO_HPP_

#define UP 'W'
#define DOWN 'S'
#define RIGHT 'D'
#define LEFT 'A'
#define TO_DEFAULT 'C'
#define SPEED_1 '1'
#define SPEED_2 '2'
#define SPEED_3 '3'
#define SPEED_4 '4'
#define SPEED_5 '5'
#define MAX_SPEED 4
#define X_AXIS 'X'
#define Y_AXIS 'Y'
#define RIGHT_LIMIT 'R'
#define LEFT_LIMIT 'L'
#define UPPER_LIMIT 'U'
#define LOWER_LIMIT 'G'

using namespace std;
using namespace LibSerial;

char connectServo (char serialName[]);
void disconnectServo ();
void servoSet (char param);
char servoGetParam (char code);

#endif /* SRC_SERVO_HPP_ */
