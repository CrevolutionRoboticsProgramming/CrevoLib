/*
 * CrevoRobot.h
 *
 *  Created on: Dec 19, 2016
 *      Author: Martin
 */

#ifndef SRC_CREVOROBOT_H_
#define SRC_CREVOROBOT_H_
#include <iostream>
#include <stdio.h>
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

#define PRAC_BOT
//#define COMP_BOT
#define NotDebug
class CrevoRobot{

private:

/*________________________________________________________________________________________________________________________________*/

		enum Controllers{
			 	 	DRIVER_CONTROLLER = 0,
			 	 	OPERATOR_CONTROLLER = 1,
		 };
/*________________________________________________________________________________________________________________________________*/

#ifdef PRAC_BOT
		enum MotorCAN{
			 	 	RIGHT_FRONT_PORT = 6,
					RIGHT_REAR_PORT = 5,
					LEFT_FRONT_PORT = 8,
					LEFT_REAR_PORT = 9,
					SHOOTER_MOTOR_A = 8,
					SHOOTER_MOTOR_B = 6,
					AGITATOR_MOTOR = 5,
					INTAKE_MOTOR = 7,
		 };

#endif /*PRAC_BOT*/

#ifdef COMP_BOT
		enum MotorCAN{
				RIGHT_FRONT_PORT = 6,
				RIGHT_REAR_PORT = 5,
				LEFT_FRONT_PORT = 8,
				LEFT_REAR_PORT = 9,
				SHOOTER_MOTOR_A = 8,
				SHOOTER_MOTOR_B = 6,
				INTAKE_MOTOR = 7,
				 };

#endif

//#else
	//	std::cout << "Need to Select Robot Configuration!!" << std:endl;
		//std::cout << "Redownload porgram with corresponding robot." << std::endl;
/*________________________________________________________________________________________________________________________________*/

		 enum MotorPWM{

		 };
/*________________________________________________________________________________________________________________________________*/

		 enum AnalogPort{
			 	 	GYRO = 1,
					POTEIOMETER = 1,
					ACCELEROMETER = 2,
		 };
/*________________________________________________________________________________________________________________________________*/

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
/*________________________________________________________________________________________________________________________________*/

		 enum SoleniodPort{
			 	 	RIGHT_DRIVE_SWITCH1 = 1,
					RIGHT_DRIVE_SWITCH2 = 2,
					LEFT_DRIVE_SWTICH1 = 3,
					LEFT_DRIVE_SWITCH2 = 4,
		 };
/*________________________________________________________________________________________________________________________________*/

public:
		 CANTalon 	  *rightFrontMotor   = NULL;
		 CANTalon 	  *rightRearMotor 	 = NULL;
		 CANTalon 	  *leftFrontMotor	 = NULL;
		 CANTalon 	  *leftRearMotor	 = NULL;
		 CANTalon 	  *intakeRoller 	 = NULL;
		 CANTalon 	  *fuelManipulator   = NULL;
		 CANTalon 	  *fuelManipulator2  = NULL;
		 CANTalon	  *agitatorMotor     = NULL;
		 CANTalon	  *hangerMotor 	     = NULL;

		 RobotDrive   *robotDrive;

		 Compressor   *compressor;

		 DigitalInput *limitSwitch1;
		 DigitalInput *limitSwitch2;

		 Encoder 	  *fuelManipulatorEncoder;
		 Encoder 	  *rightEnc;
		 Encoder 	  *leftEnc;

		 AnalogGyro   *gyro;
		 AnalogPotentiometer *pot;
		 AnalogAccelerometer *accel;

		 std::shared_ptr<NetworkTable> table;


		 //Length of current year's robot
		 const double robotLentgh = 28;
		 //Width of current year's robot
		 const double robotWidth = 32.5;
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

#endif /* SRC_CREVOROBOT_H_ */
