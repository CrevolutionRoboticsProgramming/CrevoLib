/*
 * CrevoRobot.h
 *
 *  Created on: Dec 19, 2016
 *      Author: Martin
 */

#ifndef SRC_CREVOROBOT_H_
#define SRC_CREVOROBOT_H_

#include <CANTalon.h>
#include <Talon.h>
#include <AnalogAccelerometer.h>
#include <AnalogGyro.h>
#include <AnalogPotentiometer.h>
#include <DigitalInput.h>
#include <DoubleSolenoid.h>
#include <Encoder.h>
#include <Compressor.h>
#include <RobotDrive.h>

//#include <>
#define NULL 0

class CrevoRobot{

private:

/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
		 enum Controllers{
			 	 	DRIVER_CONTROLLER = 0,
			 	 	OPERATOR_CONTROLLER = 1,
		 };
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
		 enum MotorCAN{
			 	 	RIGHT_FRONT_PORT = 1,
					RIGHT_REAR_PORT = 2,
					LEFT_FRONT_PORT = 3,
					LEFT_REAR_PORT = 4,
					FRONT_RECLUSE_MOTOR = 5,
					REAR_RECLUSE_MOTOR = 6,
					SHOOTER_MOTOR = 7,
					INTAKE_MOTOR = 8,
					//ARM_MOTOR = 7,

		 };
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

		 enum MotorPWM{

		 };
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

		 enum AnalogPort{
			 	 	GYRO = 1,
					POTEIOMETER = 1,
					ACCELEROMETER = 2,
		 };
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

		 enum DigitalPort{
			 	 	LIMIT_SWITCH_1 = 0,
					LIMIT_SWITCH_2 = 1,
					FUEL_MANIPULATOR_ENCODER_1 = 0,
					FUEL_MANIPULATOR_ENCODER_2 = 1,
					R_EN_1 = 3,
					R_EN_2 = 4,
					L_EN_1 = 5,
					L_EN_2 = 6,
		 };
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

		 enum SoleniodPort{
			 RIGHT_DRIVE_SWITCH1 = 1,
			 RIGHT_DRIVE_SWITCH2 = 2,
			 LEFT_DRIVE_SWTICH1 = 3,
			 LEFT_DRIVE_SWITCH2 = 4,

		 };
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/


public:
		 CANTalon *rightFrontMotor = NULL;
		 CANTalon *rightRearMotor = NULL;
		 CANTalon *leftFrontMotor = NULL;
		 CANTalon *leftRearMotor = NULL;
		 CANTalon *intakeRoller = NULL;
		 CANTalon *fuelManipulator = NULL;
		 CANTalon *armMotor = NULL;

		 RobotDrive *robotDrive;

		 Compressor *compressor;

		 DigitalInput *limitSwitch1;
		 DigitalInput *limitSwitch2;

		 Encoder *fuelManipulatorEncoder;
		 Encoder *rightEnc;
		 Encoder *leftEnc;

		 AnalogGyro *gyro;
		 AnalogPotentiometer *pot;
		 AnalogAccelerometer *accel;

		 //Length of current year's robot
		 const double robotLentgh = 0;
		 //Width of current year's robot
		 const double robotWidth = 0;
		 //Encoder Counts per Revolution
		 const double encoderCPR = 1024;

		 const double magicCPR = 4096;


		 /*
		  * Distance traveled = Wheel rotations * circumference
		  * or
		  * Distance traveled = (Degrees turned / 360) * circumference
		  * or
		  * Distance traveled = ( Encoder ticks / 360) * circumference
		  *
		  * which Leads to:
		  * Encoder ticks = (360 / circumference) * Distance to travel
		  */

		 //double calcdistanceperPulse = ((wheelCircumfrence ) / encoderCPR );

		 void robotInit(void);
		 CrevoRobot();
		 virtual ~CrevoRobot();
};

//} /* namespace CrevoRobot */

#endif /* SRC_CREVOROBOT_H_ */
