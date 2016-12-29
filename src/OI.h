/*
 * OI.h
 *
 *  Created on: Dec 21, 2016
 *      Author: Martin
 */

#ifndef OI_H_
#define OI_H_

#include "Joystick.h"
//namespace OI {

class OI {

private:
	Joystick *selectController;
	bool inputActive;
	bool inputSlow;

public:
	enum btn {
					kXButton = 1,
					kAButton = 2,
					kYButton = 3,
					kBButton = 0,
					kLBummber = 0,
					kRBummber = 0,
					kLTrigger = 7,
					kRTrigger = 8,
					kLStick_Button = 0,
					kRStick_Button = 0,
					kBack = 0,
					kStart = 0,
					kDPAD_UP = 0,
					kDPAD_DOWN = 0,
					kDPAD_RIGHT = 0,
					kDPAD_LEFT = 0,

		};
	enum axes {
					kRIGHT_X = 0,
					kRIGHT_Y = 1,
					kLEFT_X = 2,
					kLEFT_Y = 3,
		};

	bool controllerButton(Joystick *joystick, btn button);
	double controllerJoystick(Joystick *joystick, axes axes);
	double shape(double inValue);
	OI();
	virtual ~OI();
};

//} /* namespace OI */

#endif /* SRC_OI_H_ */
