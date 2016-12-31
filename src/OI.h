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
	enum Button {
					X = 1,
					A = 2,
					Y = 3,
					B = 4,
					LeftBummber = 0,
					RightBummber = 0,
					LeftTrigger = 7,
					RightTrigger = 8,
					LeftStick = 0,
					RrightStick = 0,
					Back = 0,
					Start = 0,
					DPADUp = 0,
					DPADDownN = 0,
					DPADRight = 0,
					DPADLeft = 0,

		};
	enum Axes {
					RIGHT_X = 0,
					RIGHT_Y = 1,
					LEFT_X = 2,
					LEFT_Y = 3,
		};

	bool controllerButton(Joystick *joystick, Button button);
	float controllerJoystick(Joystick *joystick, Axes axes);
	float shape(float inValue);
	OI();
	virtual ~OI();
};

//} /* namespace OI */

#endif /* SRC_OI_H_ */
