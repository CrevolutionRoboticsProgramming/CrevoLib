/*
 * crevoglb.h
 *
 *  Created on: Feb 19, 2017
 *      Author: Martin Smoger
 */

#ifndef SRC_CREVOGLB_H_
#define SRC_CREVOGLB_H_

#include <OI.h>
#include <CrevoRobot.h>
#include <DriveTrain.h>
#include <Vision.h>
#include <FeedBack.h>


enum AllianceSide { RED, BLUE};

enum MotorState { ON, OFF };

enum Direction {Forward, Reverse};

struct Speed { double Full = 1;
			   double ThreeQuarter = 0.75;
			   double Half = 0.5;
			   double ForwardQuarter = 0.25;
			   double Off = 0;} Speed;

double errorClip(double number , double min, double max)
{
	if(number < min) return min;
	if(number > max) return max;
	return number;
}

#endif /* SRC_CREVOGLB_H_ */
