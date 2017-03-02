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
#include <Vision.h>
#include <FeedBack.h>

#ifndef max
    #define max(a,b)    ((a) > (b) ? (a) : (b))
#endif

#ifndef min
    #define min(a,b)    ((a) < (b) ? (a) : (b))
#endif

enum AllianceSide { RED, BLUE};

enum MotorState { ON, OFF };

enum Direction {Forward, Reverse};


#endif /* SRC_CREVOGLB_H_ */
