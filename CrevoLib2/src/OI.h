/*
 * OI.h
 *
 *  Created on: Dec 21, 2016
 *      Author: Martin
 */

#ifndef OI_H_
#define OI_H_

#include <Joystick.h>
//namespace OI {

class OI {

private:
	Joystick *selectController;
	bool inputActive;
	bool inputSlow;

public:
	enum Button {
					X = 3,
					A = 1,
					Y = 4,
					B = 2,
					LeftBummber = 5,
					RightBummber = 6,
					LeftTrigger = 2,
					RightTrigger = 3,
					LeftStick = 0,
					RrightStick = 0,
					Back = 7,
					Start = 8,
					LeftJoystick = 9,
					RightJpystick = 10,
					DPADUp = 0,
					DPADDownN = 0,
					DPADRight = 0,
					DPADLeft = 0,

		};
	enum Axes {
					RIGHT_X = 5,
					RIGHT_Y = 4,
					LEFT_X = 0,
					LEFT_Y = 1,
		};

	bool controllerButton(Joystick *joystick, Button button);
	float controllerJoystick(Joystick *joystick, Axes axes);
	float shape(float inValue);
	OI();
	virtual ~OI();
};

//} /* namespace OI */

#endif /* SRC_OI_H_ */
