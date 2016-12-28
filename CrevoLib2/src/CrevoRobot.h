/*
 * CrevoRobot.h
 *
 *  Created on: Dec 19, 2016
 *      Author: Martin
 */

#ifndef SRC_CREVOROBOT_H_
#define SRC_CREVOROBOT_H_

#include "CANTalon.h"
#include "Talon.h"
#include "AnalogGyro.h"
#include "DigitalInput.h"
#include "AnalogPotentiometer.h"
#include "Encoder.h"
#include "RobotDrive.h"
//#include <>

class CrevoRobot{

private:
		 enum Controllers{
			 	 	DRIVER_CONTROLLER = 0,
			 	 	OPERATOR_CONTROLLER = 1,
		 };

		 enum MotorCAN{
			 	 	RIGHT_FRONT_PORT = 0,
					RIGHT_REAR_PORT =1,
					LEFT_FRONT_PORT = 2,
					LEFT_REAR_PORT = 3,
					ARM_MOTOR = 4,
					INTAKE_MOTOR = 5,
		 };

		 enum MotorPWM{

		 };

		 enum AnalogPort{
			 	 	GYRO = 0,
					POTEIOMETER = 1,
		 };

		 enum DigitalPort{
			 	 	LIMIT_SWITCH_1 = 0,
					LIMIT_SWITCH_2 = 1,
					R_EN_1 = 3,
					R_EN_2 = 4,
					L_EN_1 = 5,
					L_EN_2 = 6,
		 };

		 enum SoleniodPort{
		 };

public:
		 CANTalon *rightFrontMotor,
		          *rightRearMotor,
		          *leftFrontMotor,
		          *leftRearMotor,
		          *intakeRoller,
		          *armMotor;

		 RobotDrive *robotDrive;

		 DigitalInput *limitSwitch1,
		 	 	 	  *limitSwitch2;

		 Encoder *rightEnc,
		 	 	 *leftEnc;

		 AnalogGyro *gyro;

		 AnalogPotentiometer *pot;

    void robotInit(void);
	CrevoRobot();
	virtual ~CrevoRobot();
};

//} /* namespace CrevoRobot */

#endif /* SRC_CREVOROBOT_H_ */
