/*
 * stateMachine.hpp
 *
 *  Created on: 21/nov/2015
 *      Author: Dazzi Martino,
 *      		Mele Leandro Julian,
 *      		Pilotto Alessandro
 *
 *              Università degli Studi di Udine,
 *              Via delle Scienze 208,
 *              Udine, Italy
 */

#ifndef SRC_STATEMACHINE_HPP_
#define SRC_STATEMACHINE_HPP_

using namespace std;
using namespace cv;

#define FRAME_LIMIT_MAX 9

enum state {
				INIT = 0,
				CAMERA_MODE,
				MANUAL,
				TRACKING
			};

char SM_Handler (VideoCapture &cap, char serialName[]);
static void onMouse(int event, int x, int y, int, void*);
void showInfo (Mat frame);
void position_widget (Mat frame);

#endif /* SRC_STATEMACHINE_HPP_ */
