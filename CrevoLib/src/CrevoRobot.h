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
#include <string>
#include <CANTalon.h>
#include <Solenoid.h>
#include <Talon.h>
#include <AnalogAccelerometer.h>
#include <AnalogGyro.h>
#include <AnalogPotentiometer.h>
#include <SmartDashboard/SmartDashboard.h>
#include <DigitalInput.h>
#include <DoubleSolenoid.h>
#include <Solenoid.h>
#include <Encoder.h>
#include <Compressor.h>
#include <RobotDrive.h>

#define ROBOT_1

#define NOT_DEBUG

#define SHOOTER_BACKUP

class CrevoRobot{

private:

/*________________________________________________________________________________________________________________________________*/

		enum Controllers{
			 	 	DRIVER_CONTROLLER = 0,
			 	 	OPERATOR_CONTROLLER = 1,
		 };
/*________________________________________________________________________________________________________________________________*/

#ifdef ROBOT_1

#define RIGHT_MULTIPLER 1
#define  LEFT_MULTIPLER 1
		enum MotorCAN{
			 	 	RIGHT_FRONT_PORT = 1,
					RIGHT_REAR_PORT = 14,
					LEFT_FRONT_PORT = 28,
					LEFT_REAR_PORT = 27,
					FUEL_SHOOTER_MASTER = 12,
					FUEL_SHOOTER_SLAVE = 10,
					AGITATOR_MOTOR = 2,
					INTAKE_MOTOR = 29,
					HANGER_MOTOR = 11,

		 };
#endif /*PRAC_BOT*/

#ifndef ROBOT_1
#define RIGHT_MULTIPLER   1
#define  LEFT_MULTIPLER   0.98
		enum MotorCAN{
				RIGHT_FRONT_PORT = 17,
				RIGHT_REAR_PORT = 19,
				LEFT_FRONT_PORT = 16,
				LEFT_REAR_PORT = 18,
				FUEL_SHOOTER_MASTER = 20,
				FUEL_SHOOTER_SLAVE = 21,
				AGITATOR_MOTOR = 22,
				HANGER_MOTOR = 23,
				INTAKE_MOTOR = 24,
				 };

#endif

//#else
	//	std::cout << "Need to Select Robot Configuration!!" << std:endl;';;
		//std::cout << "Redownload porgram with corresponding robot." << std::endl;
/*________________________________________________________________________________________________________________________________*/

		 enum MotorPWM{

		 };
/*________________________________________________________________________________________________________________________________*/

		 enum AnalogPort{
			 	 	GYRO = 0,
					POTEIOMETER = 1,
					ACCELEROMETER = 2,
		 };
/*________________________________________________________________________________________________________________________________*/

		 enum DigitalPort{
			 	 	GEAR_SENSOR = 0,
					LIMIT_SWITCH_2 = 1,
					FUEL_MANIPULATOR_ENCODER_1 = 0,
					FUEL_MANIPULATOR_ENCODER_2 = 1,
					R_EN_1 = 2,
					R_EN_2 = 3,
					L_EN_1 = 0,
					L_EN_2 = 1,
		 };
/*________________________________________________________________________________________________________________________________*/

		 enum SolenoidPort{
			 	 	RIGHT_DRIVE_SWITCH1 = 1,
					RIGHT_DRIVE_SWITCH2 = 2,
					LEFT_DRIVE_SWTICH1 = 3,
					LEFT_DRIVE_SWITCH2 = 4,
					RED_LED_PORT = 0,
					BLUE_LED_PORT = 1,
					GREEN_LED_PORT = 2,
		 };
/*________________________________________________________________________________________________________________________________*/

public:
		 CANTalon 	  *rightFrontMotor   = NULL;
		 CANTalon 	  *rightRearMotor 	 = NULL;
		 CANTalon 	  *leftFrontMotor	 = NULL;
		 CANTalon 	  *leftRearMotor	 = NULL;
		 CANTalon 	  *intakeRoller 	 = NULL;
		 CANTalon 	  *fuelShooterMaster = NULL;
		 CANTalon 	  *fuelShooterSlave  = NULL;
		 CANTalon	  *agitatorMotor     = NULL;
		 CANTalon	  *hangerMotor 	     = NULL;

		 RobotDrive   *robotDrive;

		 Compressor   *compressor;

		 Solenoid 	  *RedLED;
		 Solenoid 	  *BlueLED;
		 Solenoid 	  *GreenLED;

		 DigitalInput *gearSensor;
		 DigitalInput *limitSwitch2;

		 Encoder 	  *fuelShooterMasterEncoder;
		 Encoder 	  *rightEnc;
		 Encoder 	  *leftEnc;

		 AnalogGyro   *gyro;
		 AnalogPotentiometer *pot;
		 AnalogAccelerometer *accel;

		 //Length of current year's robot
		 const double robotLentgh = 28;
		 //Width of current year's robot
		 const double robotWidth = 32.5;
		 //Encoder Counts per Revolution
		 const double encoderCPR = 1024;

		 const double magicCPR = 4096;

		 const double wheelCircumference = 6;

		 /*/
		  * These values hold variables for the PID controller
		  * Are adjusted in the preference section of the Smart Dashboard
		 /*/
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

		 double calcdistanceperPulse = ((wheelCircumference ) / encoderCPR );

		 void robotInit(void);
		 CrevoRobot();
		 virtual ~CrevoRobot();
};

#endif /* SRC_CREVOROBOT_H_ */
