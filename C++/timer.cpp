/*
 * timer.cpp
 *
 *  Created on: 08/mag/2015
 *      Author: alessandro
 */

/* INCLUDES */
#include <iostream>
#include <ctime>
#include "timer.hpp"

/* NAMESPACES */
using namespace std;


/* CODE */
void getTime (char buffer[])
{
	 time_t rawtime;
	 struct tm * timeinfo;

	 time (&rawtime);
	 timeinfo = localtime(&rawtime);
     strftime(buffer, 80, "%d-%m-%y_%I:%M:%S.jpg", timeinfo);

	return;
}
