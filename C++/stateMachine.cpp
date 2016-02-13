/*
 * stateMachine.cpp
 *
 *  Created on: 21/nov/2015
 *      Author: Dazzi Martino,
 *      		Mele Leandro Julian,
 *      		Pilotto Alessandro
 *
 *              Università degli Studi di Udine,
 *              Via delle Scienze 208,
 *              Udine, Italy
 *
 *  For informations regarding MIL tracking algorithm see:
 *  [MIL] B Babenko, M-H Yang, and S Belongie, Visual Tracking with Online Multiple Instance Learning, In CVPR, 2009
 */

#include <stdio.h>
#include <string>
#include <cv.h>
#include "SerialStream.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/tracking.hpp"
#include "opencv2/tracking/tracker.hpp"
#include "opencv2/tracking/tracking.hpp"

#include "main.hpp"
#include "stateMachine.hpp"
#include "servo.hpp"
#include "timer.hpp"

// TODO: define DEBUG if you don't have servos
//#define DEBUG

using namespace std;
using namespace cv;
using namespace LibSerial;

const string windowName = "Video Follower";
int state = INIT;

int WEBCAM_WIDTH = 640;
int WEBCAM_HEIGHT = 480;

const int R = 70;
int x_left_rel_point = 0;
int y_left_rel_point= 0;
int x_right_rel_point= 0;
int y_right_rel_point= 0;
int x_up_rel_point= 0;
int y_up_rel_point= 0;
int x_low_rel_point= 0;
int y_low_rel_point= 0;


char xPos = 0;
char yPos = 0;
char step = 1;

char rightLimit = 0;
char leftLimit = 0;
char upperLimit = 0;
char lowerLimit = 0;

#ifndef DEBUG
	bool servoRun = false;
#else
	bool servoRun = true;
#endif

bool trackerInitialized = false;
bool referenceSelected = false;
bool startReferenceSelection = false;
bool objSelected = false;
bool startObjSelection = false;
bool showOptions = false;

Ptr<Tracker> tracker;
Rect2d objBox;
Point objCenter;
Rect2d referenceBox;
Mat image;

Scalar color(255, 0, 0);

char SM_Handler (VideoCapture &cap, char serialName[])
{
	char error = ERROR_NONE;
	char time[80];
	int frameCNT = 0;
	int frameLimit = 0;
	int speed = 0;

	bool flipped = false;
	bool inverted = false;

	/* Create a window */
	namedWindow(windowName);
	setMouseCallback(windowName, onMouse, 0);

	createTrackbar("Interval", windowName, &frameLimit, FRAME_LIMIT_MAX);
	createTrackbar("Speed", windowName, &speed, MAX_SPEED);

	Mat logo = imread("logo.jpg");
	if (true == logo.empty())
		state = CAMERA_MODE;

	while (1)
	{
		Mat frame;
		cap >> frame;

		resize(frame, image, Size(WEBCAM_WIDTH, WEBCAM_HEIGHT));

		if (true == flipped)
			flip(image, image, 1);

		char c = (char)(waitKey(1));

		if ((c == 'o') || (c == 'O'))
			showOptions = !showOptions;

		if (INIT == state)
		{
			if (c == 27)
				break;
			else if (c == ' ')
				state = CAMERA_MODE;

			imshow(windowName, logo);
		}
		else if (CAMERA_MODE == state)
		{

			if (c == 27)
			{
				if (true == servoRun)
				{
					disconnectServo();
				}

				break;
			}

#ifndef DEBUG
			if (((c == 's') || (c == 'S')) && (servoRun == false))
			{
				if (ERROR_NONE != (error = connectServo(serialName)))
				{
					break;
				}
				//sleep(2); // TODO: Try to remove it
				servoSet(speed + 1 + '0');
				step = speed + 1;
				leftLimit = servoGetParam(LEFT_LIMIT);
				rightLimit = servoGetParam(RIGHT_LIMIT);
				upperLimit = servoGetParam(UPPER_LIMIT);
				lowerLimit = servoGetParam(LOWER_LIMIT);
				xPos = servoGetParam(X_AXIS);
				yPos = servoGetParam(Y_AXIS);
				servoRun = true;

				// setting widgets limits lines

				double rad_leftLimit = (leftLimit*3.1416)/180;
				y_left_rel_point = R*sin(0.5*3.1416+rad_leftLimit);
			    x_left_rel_point = R*cos(0.5*3.1416+rad_leftLimit);

			    double rad_rightLimit = (rightLimit*3.1416)/180;
			    y_right_rel_point = R*sin(0.5*3.1416+rad_rightLimit);
			    x_right_rel_point = R*cos(0.5*3.1416+rad_rightLimit);

			    double rad_upperLimit = (upperLimit*3.1416)/180;
			    y_up_rel_point = R*sin(3.1416+rad_upperLimit);
			    x_up_rel_point = R*cos(3.1416+rad_upperLimit);

			    double rad_lowerLimit = (lowerLimit*3.1416)/180;
			    y_low_rel_point = R*sin(3.1416+rad_lowerLimit);
			    x_low_rel_point = R*cos(3.1416+rad_lowerLimit);

			}
			else if (((c == 's') || (c == 'S')) && (servoRun == true))
			{
				disconnectServo();
				servoRun = false;
			}
#endif

			else if (((c == 'm') || (c == 'M')) && (servoRun == true))
				state = MANUAL;
			else if (((c == 't') || (c == 'T')) && (servoRun == true))
			{
				if(NULL == (tracker = Tracker::create("MIL")))
				{
					error = ERROR_TRACKER;
					break;
				}
				state = TRACKING;
			}

#ifndef DEBUG
			else if (((c == 'v') || (c == 'V')) && (servoRun == true))
			{
				servoSet(speed + 1 + '0');
				step = speed + 1;
			}
#endif

			else if ((c == 'f') || (c == 'F'))
			{
				flipped = !flipped;
			}
			else if ((c == 'p') || (c == 'P'))
			{
				getTime(time);
				string strTime(time);
				imwrite(strTime, image);
			}
			else if ((c == 'R') || (c == 'r'))
				inverted = !inverted;

			showInfo(image);
			imshow(windowName, image);
		}
		else if (MANUAL == state)
		{

			if (c == 27)
				state = CAMERA_MODE;

#ifndef DEBUG
				else if ((c == 'w') || (c == 'W'))
				{
					servoSet(UP);
					if ((yPos + step) <= upperLimit)
						yPos += step;
				}
				else if ((c == 's') || (c == 'S'))
				{
					servoSet(DOWN);
					if ((yPos - step) >= lowerLimit)
						yPos -= step;
				}
				else if ((c == 'a') || (c == 'A'))
				{
					if (false == inverted)
					{
						servoSet(LEFT);
						if ((xPos + step) <= leftLimit)
							xPos += step;
					}
					else
					{
						servoSet(RIGHT);
						if ((xPos - step) >= rightLimit)
							xPos -= step;
					}
				}
				else if ((c == 'd') || (c == 'D'))
				{
					if (false == inverted)
					{
						servoSet(RIGHT);
						if ((xPos - step) >= rightLimit)
							xPos -= step;
					}
					else
					{
						servoSet(LEFT);
						if ((xPos + step) <= leftLimit)
							xPos += step;
					}
				}
				else if ((c == 'c') || (c == 'C'))
				{
					servoSet(TO_DEFAULT);
					xPos = 0;
					yPos = 0;
				}
				else if ((c == 'v') || (c == 'V'))
				{
					servoSet(speed + 1 + '0');
					step = speed + 1;
				}
#endif


			else if ((c == 'f') || (c == 'F'))
			{
				flipped = !flipped;
			}
			else if ((c == 'p') || (c == 'P'))
			{
				getTime(time);
				string strTime(time);
				imwrite(strTime, image);
			}
			else if ((c == 'R') || (c == 'r'))
				inverted = !inverted;

			showInfo(image);
			imshow(windowName, image);
		}
		else if (TRACKING == state)
		{
			if (c == 27)
			{
				tracker->clear();
				trackerInitialized = false;
				referenceSelected = false;
				startReferenceSelection = false;
				objSelected = false;
				startObjSelection = false;

#ifndef DEBUG
				servoSet(TO_DEFAULT);
				xPos = 0;
				yPos = 0;
#endif

				state = CAMERA_MODE;
			}

#ifndef DEBUG

			else if (((c == 'v') || (c == 'V')) && (servoRun == true))
			{
				servoSet(speed + 1 + '0');
				step = speed + 1;
			}
#endif

			if(true == objSelected)
			{
				if(false == trackerInitialized)
				{
					if(!tracker->init(image, objBox))
					{
						error = ERROR_TRACKER;
						break;
					}
					trackerInitialized = true;
				}
				else
				{
					if( tracker->update(image, objBox))
					{
						frameCNT++;

						/* Find objBox center */
						objCenter.x = objBox.x + objBox.width/2;
						objCenter.y = objBox.y + objBox.height/2;
#ifndef DEBUG
						if ((frameLimit + 1) <= frameCNT)
						{
							/* Move servos */
							if (objCenter.x < referenceBox.x)
							{
								if (false == inverted)
								{
									servoSet(LEFT);
									if ((xPos + step) <= leftLimit)
										xPos += step;
								}
								else
								{
									servoSet(RIGHT);
									if ((xPos - step) >= rightLimit)
										xPos -= step;
								}
							}
							if (objCenter.x > referenceBox.x + referenceBox.width)
							{
								if (false == inverted)
								{
									servoSet(RIGHT);
									if ((xPos - step) >= rightLimit)
										xPos -= step;
								}
								else
								{
									servoSet(LEFT);
									if ((xPos + step) <= leftLimit)
										xPos += step;
								}
							}
							if (objCenter.y < referenceBox.y)
							{
								servoSet(UP);
								if ((yPos + step) <= upperLimit)
									yPos += step;
							}
							if (objCenter.y > referenceBox.y + referenceBox.height)
							{
								servoSet(DOWN);
								if ((yPos - step) >= lowerLimit)
									yPos -= step;
							}

							frameCNT = 0;
						}
#endif
					}
				}
			}

			showInfo(image);
			imshow(windowName, image);
		}
	}

	return error;
}


static void onMouse(int event, int x, int y, int, void*)
{
	if (TRACKING == state)
	{
		if(false == objSelected)
		{
			switch (event)
			{
				case EVENT_LBUTTONDOWN:	if (false == referenceSelected)
										{
											startReferenceSelection = true;
											referenceBox.x = x;
											referenceBox.y = y;
											referenceBox.width = referenceBox.height = 0;
										}
										else
										{
											startObjSelection = true;
											objBox.x = x;
											objBox.y = y;
											objBox.width = objBox.height = 0;
										}
										break;

				case EVENT_LBUTTONUP:	if (false == referenceSelected)
										{
											referenceBox.width = std::abs(x - referenceBox.x);
											referenceBox.height = std::abs(y - referenceBox.y);
											referenceSelected = true;
										}
										else
										{
											objBox.width = std::abs(x - objBox.x);
											objBox.height = std::abs(y - objBox.y);
											if ((objBox.width <= referenceBox.width) && (objBox.height <= referenceBox.height))
												objSelected = true;
											else
											{
												printf("Object box must be smaller than reference box.\nPlease, try again.\n");
												startObjSelection = false;
												startReferenceSelection = false;
												referenceSelected = false;
											}
										}
										break;

				case EVENT_MOUSEMOVE:	if((true == startReferenceSelection) && (false == referenceSelected))
										{
											rectangle(image, Point((int)referenceBox.x, (int)referenceBox.y), Point(x, y), Scalar(0, 0, 0), 3, 1);
											imshow(windowName, image);
										}
										else if((true == startObjSelection) && (false == objSelected))
										{
											rectangle(image, Point((int)objBox.x, (int)objBox.y), Point(x, y), Scalar(255, 0, 0), 2, 1);
											imshow(windowName, image);
										}
										break;
			}
		}
	}
}

void showInfo (Mat frame)
{
	if (CAMERA_MODE == state)
	{
		putText(frame, "ESC exit", Point(20,20), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255,0,0), 1.0);

		if (true == servoRun)
		{
			if (true == showOptions)
			{
				putText(frame, "S disconnect servos", Point(20,40), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255,0,0), 1.0);
				putText(frame, "M manual mode", Point(20,60), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255,0,0), 1.0);
				putText(frame, "T tracking", Point(20,80), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255,0,0), 1.0);
				putText(frame, "O hide options", Point(20,100), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255,0,0), 1.0);
				putText(frame, "F flip", Point(20, 120), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255,0,0), 1.0);
				putText(frame, "P take a snapshot", Point(20, 140), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255,0,0), 1.0);
				putText(frame, "R invert X-axis", Point(20,160), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255,0,0), 1.0);
				putText(frame, "V set speed", Point(20,180), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255,0,0), 1.0);
			}
			else
			{
				putText(frame, "S disconnect servos", Point(20,40), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255,0,0), 1.0);
				putText(frame, "M manual mode", Point(20,60), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255,0,0), 1.0);
				putText(frame, "T tracking", Point(20,80), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255,0,0), 1.0);
				putText(frame, "O show options", Point(20,100), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255,0,0), 1.0);
			}
		}
		else
		{
				putText(frame, "S connect servos", Point(20,40), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255,0,0), 1.0);
		}

		putText(frame, "STANDARD", Point(20, WEBCAM_HEIGHT-20), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255,0,0), 1.0);
	}
	else if (MANUAL == state)
	{
		position_widget (frame);
		putText(frame, "ESC standard camera", Point(20,20), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255,0,0), 1.0);

		if (true == showOptions)
		{
			putText(frame, "WASD move servos", Point(20,40), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255,0,0), 1.0);
			putText(frame, "C default position", Point(20,60), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255,0,0), 1.0);
			putText(frame, "O hide options", Point(20, 80), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255,0,0), 1.0);
			putText(frame, "F flip", Point(20, 100), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255,0,0), 1.0);
			putText(frame, "P take a snapshot", Point(20, 120), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255,0,0), 1.0);
			putText(frame, "R invert X-axis", Point(20,140), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255,0,0), 1.0);
			putText(frame, "V set speed", Point(20,160), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255,0,0), 1.0);

		}
		else
		{
			putText(frame, "WASD move servos", Point(20,40), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255,0,0), 1.0);
			putText(frame, "C default position", Point(20,60), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255,0,0), 1.0);
			putText(frame, "O show options", Point(20, 80), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255,0,0), 1.0);
		}
		putText(frame, "MANUAL", Point(20, WEBCAM_HEIGHT-20), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255,0,0), 1.0);
	}
	else if (TRACKING == state)
	{
		position_widget (frame);
		putText(frame, "ESC standard camera", Point(20,20), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255,0,0), 1.0);

		if(false == showOptions)
			putText(frame, "O show options", Point(20, 40), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255,0,0), 1.0);
		else
		{
			putText(frame, "O hide options", Point(20, 40), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255,0,0), 1.0);
			putText(frame, "V set speed", Point(20,60), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255,0,0), 1.0);
		}


		if (true == referenceSelected)
		{
			/* Reference rectangle must be centered */
			if (referenceBox.x + referenceBox.width/2 > WEBCAM_WIDTH/2)
				referenceBox.x = referenceBox.width/2 - WEBCAM_WIDTH/2;
			else
				referenceBox.x = WEBCAM_WIDTH/2 - referenceBox.width/2;

			if (referenceBox.y + referenceBox.height/2 > WEBCAM_HEIGHT/2)
				referenceBox.y = referenceBox.height/2 - WEBCAM_HEIGHT/2;
			else
				referenceBox.y = WEBCAM_HEIGHT/2 - referenceBox.height/2;

			rectangle(image, Point((int)referenceBox.x, (int)referenceBox.y), Point((int)referenceBox.x + referenceBox.width, (int)referenceBox.y + referenceBox.height), Scalar(0, 0, 0), 3, 1);
		}

		if (false == referenceSelected)
			putText(frame, "TRACKING: Draw a reference rectangle", Point(20, WEBCAM_HEIGHT-20), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255,0,0), 1.0);
		else if ((true == referenceSelected) && (false == objSelected))
		 	putText(frame, "TRACKING: Draw a rectangle around the object you want to track", Point(20, WEBCAM_HEIGHT-20), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255,0,0), 1.0);
		else
			putText(frame, "TRACKING", Point(20, WEBCAM_HEIGHT-20), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255,0,0), 1.0);

		if ((objCenter.x < referenceBox.x) || (objCenter.x > referenceBox.x + referenceBox.width) ||
			(objCenter.y < referenceBox.y) || (objCenter.y > referenceBox.y + referenceBox.height))
		{
			color = Scalar(0, 0, 255);
		}
		else
		{
			color = Scalar(0, 255, 0);
		}

		if(true == objSelected)
			rectangle(image, objBox, color, 2, 1);

	}
}

void position_widget (Mat frame)
{
	// Showing widget for lateral position of the servos

	double horiz_theta;
	int horiz_x_arrow, horiz_y_arrow, horiz_theta_deg;

	horiz_theta_deg = xPos + 90;
	horiz_theta = (horiz_theta_deg*3.1416)/180;

	horiz_y_arrow = R*sin(horiz_theta);
	horiz_x_arrow = R*cos(horiz_theta);

	line(frame, Point(WEBCAM_WIDTH - 100, WEBCAM_HEIGHT- 15),
				Point((WEBCAM_WIDTH-100), (WEBCAM_HEIGHT- 15)- R),
				Scalar(255,255,255), 1, 8, 0);

	// LIMITATIONS
	line(frame, Point(WEBCAM_WIDTH - 100, WEBCAM_HEIGHT- 15),
				Point((WEBCAM_WIDTH-100 + x_left_rel_point), (WEBCAM_HEIGHT- 15 - y_left_rel_point)),
				Scalar(100,100,100), 1, 8, 0);

	line(frame, Point(WEBCAM_WIDTH - 100, WEBCAM_HEIGHT- 15),
				Point((WEBCAM_WIDTH-100)+ x_right_rel_point, (WEBCAM_HEIGHT- 15) - y_right_rel_point),
				Scalar(100,100,100), 1, 8, 0);

	circle(frame, Point(WEBCAM_WIDTH - 100, WEBCAM_HEIGHT- 15),
				  R, Scalar(100,100,100),1,8,0);

	arrowedLine(frame, 	Point(WEBCAM_WIDTH - 100, WEBCAM_HEIGHT- 15),
						Point((WEBCAM_WIDTH-100)+ horiz_x_arrow, (WEBCAM_HEIGHT- 15)- horiz_y_arrow),
						Scalar(0, 190, 0), 2, CV_AA, 0, 0.1);

	string horiz_angle = "Horiz. Angle " + to_string(xPos);
	putText(frame, horiz_angle, Point(WEBCAM_WIDTH-170, WEBCAM_HEIGHT-15-70-10),
			FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255,0,0), 1.0);

	// Showing widget for vertical position of the servos

	double vertical_theta;
	int vertical_x_arrow, vertical_y_arrow;

	vertical_theta = (yPos*3.1416)/180;

	vertical_y_arrow = R*sin(vertical_theta);
	vertical_x_arrow = R*cos(vertical_theta);

	line(frame, Point(WEBCAM_WIDTH - 65, WEBCAM_HEIGHT- 135),
				Point((WEBCAM_WIDTH-65)-69, (WEBCAM_HEIGHT- 135)),
				Scalar(255,255,255), 1, 8, 0);

	// LIMITATIONS
	line(frame, Point(WEBCAM_WIDTH - 65, WEBCAM_HEIGHT - 135),
				Point((WEBCAM_WIDTH-65 + x_low_rel_point), (WEBCAM_HEIGHT- 135) + y_low_rel_point),
				Scalar(100,100,100), 1, 8, 0);

	line(frame, Point(WEBCAM_WIDTH - 65, WEBCAM_HEIGHT - 135),
				Point((WEBCAM_WIDTH-65)+ x_up_rel_point, (WEBCAM_HEIGHT- 135) + y_up_rel_point),
				Scalar(100,100,100), 1, 8, 0);

	arrowedLine(frame, 	Point(WEBCAM_WIDTH - 65, WEBCAM_HEIGHT - 135),
						Point((WEBCAM_WIDTH- 65)- vertical_x_arrow, (WEBCAM_HEIGHT- 135)- vertical_y_arrow),
						Scalar(0, 190, 0), 2, CV_AA, 0, 0.1);

	string vertical_angle = "Vertical angle " + to_string(yPos);
	putText(frame, vertical_angle, Point(WEBCAM_WIDTH-180, WEBCAM_HEIGHT-190),
			FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255,0,0), 1.0);
}
