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
	inputActive = (abs(inValue) > 0.1);
	inputSlow = (abs(inValue) < 0.8);

	float sign = (inValue);

	if(inputActive)
	{
		if (inputSlow)
		{
			float output = sign * (abs(inValue) * 4.0/7.0 - 0.4/7.0);
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
		return 0;
	}
}



bool OI::controllerButton(Joystick *joystick, Button button)
{
	// selectController = jsStick;
	 return (joystick->GetRawButton(button));
}

float OI::controllerJoystick(Joystick *joystick, Axes axes)
{
	return (joystick->GetRawAxis(axes));
}

void OI::toggleAction(bool Pressed, CANTalon *_motor, double speed)
{
	if(Pressed && !lastToggled)
	{
		toggled = !toggled;

		if(toggled)
			_motor->Set(speed);
		else
			_motor->Set(0.0);
		_motor->Set(0.0);
	}
	lastToggled = Pressed;
}

//}
