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

double OI::shape(double inValue)
{
	inputActive = (abs(inValue) > 0.1);
		inputSlow = (abs(inValue) < 0.8);

		double sign = (inValue);

		if(inputActive) {
			if (OI::inputSlow) {
				double output = sign * (abs(inValue));
				return output;
			} else {
				double output = sign * (abs(inValue));
				return output;
			}
		} else {
			return 0;
		}
}



bool OI::controllerButton(Joystick *jStick, btn button)
{
	// selectController = jsStick;
	 return (jStick->GetRawButton(button));
}

double OI::controllerJoystick(Joystick *joystick, axes axes)
{
	return (joystick->GetRawAxis(axes));
}

//}
