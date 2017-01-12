/*
 * CrevoRobot.cpp
 *
 *  Created on: Dec 19, 2016
 *      Author: Martin
 */

#include <CrevoRobot.h>

CrevoRobot::CrevoRobot(){

}

CrevoRobot::~CrevoRobot(){

}
//CrevoRobot::~CrevoRobot() {
//	// TODO Auto-generated destructor stub
//}

void CrevoRobot::robotInit(void){

	//---------------Drive MotorControllers-----------------------
	leftFrontMotor = new CANTalon(MotorCAN::LEFT_FRONT_PORT);
	leftRearMotor = new CANTalon(MotorCAN::LEFT_REAR_PORT);
	rightFrontMotor = new CANTalon(MotorCAN::RIGHT_FRONT_PORT);
	rightRearMotor = new CANTalon(MotorCAN::RIGHT_REAR_PORT);
	//------------------------------------------------------------

	//---------------Manipulators MotorControllers----------------
	intakeRoller = new CANTalon(MotorCAN::INTAKE_MOTOR);
	//------------------------------------------------------------

	//---------------Configure MotorControlers--------------------
	leftFrontMotor->SetInverted(true);
	leftRearMotor->SetInverted(true);
	rightFrontMotor->SetInverted(false);
	rightRearMotor->SetInverted(false);

	leftFrontMotor->SetP(0.3);
	leftFrontMotor->SetI(0.5);

	//------------------------------------------------------------

	//-------------------RobotDrive-------------------------------
	robotDrive =  new RobotDrive(leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor);
	//------------------------------------------------------------


	//--------------Configure Sensors-----------------------------
	gyro = new AnalogGyro(AnalogPort::GYRO);
	accel = new AnalogAccelerometer(AnalogPort::ACCELEROMETER);

	//------------------------------------------------------------


}

//}
