/*
 * FeedBack.cpp
 *
 *  Created on: Feb 12, 2017
 *      Author: Martin Smoger
 */

#include <FeedBack.h>

FeedBack::FeedBack() {
	// TODO Auto-generated constructor stub

}

FeedBack::~FeedBack() {
	// TODO Auto-generated destructor stub
}

bool FeedBack::gearIndicator(void)
{
	if(crvbot.gearSensor->Get())
	{
		elapsedRumble->Start();
	}
	return true;
}
bool FeedBack::endGameIndicator(void)
{
	return true;
}
