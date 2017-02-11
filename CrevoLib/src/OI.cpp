/*
 * OI.cpp
 *
 *  Created on: Dec 21, 2016
 *      Author: Martin
 */

#include <OI.h>
#include <Math.h>

//namespace OI {
OI::OI() {
	// TODO Auto-generated constructor stub

}

OI::~OI() {
	// TODO Auto-generated destructor stub
}

float OI::shape(float inValue)
{
	int sign;
	inputActive = (abs(inValue) > 0.1);
	inputSlow = (abs(inValue) < 0.8);

	if(inValue > 0) sign = 1;
	if(inValue < 0) sign = -1;

	if(inputActive)
	{
		if (inputSlow)
		{
			float output = sign * (abs(inValue) * 0.65);
			return output;
		}
		else
		{
			float output = sign * (abs(inValue) * 3.0 - 2.0);
			return output;
		}
	}

	else
	{
		return 0.0;
	}
}



bool OI::controllerButton(Joystick *_joystick, Button button)
{
	// selectController = jsStick;
	 return (_joystick->GetRawButton(button));
}

float OI::controllerJoystick(Joystick *_joystick, Axes axes)
{
	return (_joystick->GetRawAxis(axes));
}

void OI::toggleAction(bool Pressed, CANTalon *_motor, double speed)
{
	if(Pressed && !lastToggled)
	{
		toggled = !toggled;

		if(toggled) _motor->Set(speed);
		else    	_motor->Set(0.0);
	}
	lastToggled = Pressed;
}

//}
