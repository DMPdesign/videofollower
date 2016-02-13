/*
 * main.cpp
 *
 *  Created on: 20/nov/2015
 *      Author: Dazzi Martino,
 *      		Mele Leandro Julian,
 *      		Pilotto Alessandro
 *
 *              Università degli Studi di Udine,
 *              Via delle Scienze 208,
 *              Udine, Italy
 */

#include <stdio.h>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"

#include "main.hpp"
#include "stateMachine.hpp"

using namespace cv;

int main (int argc, char *argv[])
{
	char error = ERROR_NONE;

	printf("\nVIDEO FOLLOWER\nver 0.5\n\n");
	printf("Dazzi Martino, Mele Leandro Julian, Pilotto Alessandro\n");
	printf("Universita' degli Studi di Udine, Via delle Scienze 208, Udine, Italy\n\n");

	/* Check if the launch was correct */
	if (argc != 3)
	{
		if (argc < 3)
		{
			printf ("Serial port and camera address must be specified.\n");
			printf ("./videoFollower <serial> <camera>\n");
		}
		else
			printf ("Too many arguments for program VIDEO FOLLOWER.\n");

		error = ERROR_PARAM;
		printf ("Program terminated with error %d.\n", error);
		return error;
	}

	/* WebCam setup */
	VideoCapture cap(atoi(argv[2]));
	if (!cap.isOpened())
	{
		error = ERROR_CAM;
		printf("WebCam not found\n");
		printf("Program terminated with ERROR %d!\n", error);
		return error;
	}

	/* Start the State Machine */
	error = SM_Handler(cap, argv[1]);

	if (error != ERROR_NONE)
	{
		printf("Program terminated with ERROR %d!\n", error);
	}
	else
		printf("Program terminated with SUCCESS!\n");

	return error;
}
