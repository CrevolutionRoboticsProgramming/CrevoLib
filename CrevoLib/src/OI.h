/*
 * OI.h
 *
 *  Created on: Dec 21, 2016
 *      Author: Martin
 */

#ifndef OI_H_
#define OI_H_

#include <Joystick.h>
#include <CANTalon.h>

class OI {

private:
	Joystick *selectController;
	bool inputActive;
	bool inputSlow;
	bool stillPressed;
	bool toggled = false;
	bool lastToggled = false;

public:
	enum Button {
					X = 3,
					A = 1,
					Y = 4,
					B = 2,
					LeftBumber = 5,
					RightBumber = 6,
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
					RIGHT_X = 4,
					RIGHT_Y = 5,
					LEFT_X = 0,
					LEFT_Y = 1,
		};

	bool controllerButton(Joystick *joystick, Button button);
	float controllerJoystick(Joystick *joystick, Axes axes);
	float shape(float inValue);
	void toggleAction(bool Pressed, CANTalon *_motor, double speed);
	void whilePressedAction(bool forwardPressed, bool reversedPressed, CANTalon *_motor, double speed);
	OI();
	virtual ~OI();
};

//} /* namespace OI */

#endif /* SRC_OI_H_ */
