/*
 * FeedBack.h
 *
 *  Created on: Feb 12, 2017
 *      Author: Martin Smoger
 */

#ifndef SRC_FEEDBACK_H_
#define SRC_FEEDBACK_H_

#include <CrevoRobot.h>
#include <Timer.h>

class FeedBack {
private:
	double gearDetectedTime = 0.5;
	double endGameTime 		= 0.1;

public:
	CrevoRobot crvbot;
	Timer *elapsedRumble;

	bool gearDetected = false;
	bool endGame      = false;

	bool gearIndicator(void);
	bool endGameIndicator(void);
	FeedBack();
	virtual ~FeedBack();
};

#endif /* SRC_FEEDBACK_H_ */
