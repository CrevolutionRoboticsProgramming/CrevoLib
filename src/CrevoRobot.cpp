/*
 * CrevoRobot.cpp
 *
 *  Created on: Dec 19, 2016
 *      Author: Martin
 */

#include "CrevoRobot.h"

CrevoRobot::CrevoRobot(){

}

CrevoRobot::~CrevoRobot(){

}
//CrevoRobot::~CrevoRobot() {
//	// TODO Auto-generated destructor stub
//}

void CrevoRobot::robotInit(void){

	rightEnc = new Encoder(DigitalPort::R_EN_1, DigitalPort::R_EN_2, false, Encoder::k4X);
	leftEnc = new Encoder(DigitalPort::L_EN_1, DigitalPort::L_EN_2,true,Encoder:: k4X);

	rightFrontMotor = new CANTalon(MotorCAN::RIGHT_FRONT_PORT);
	rightRearMotor = new CANTalon(MotorCAN::RIGHT_REAR_PORT);
	leftFrontMotor = new CANTalon(MotorCAN::LEFT_FRONT_PORT);
	leftRearMotor = new CANTalon(MotorCAN::LEFT_REAR_PORT);

	intakeRoller = new CANTalon(MotorCAN::INTAKE_MOTOR);
	armMotor = new CANTalon(MotorCAN::ARM_MOTOR);

	gyro = new AnalogGyro(AnalogPort::GYRO);

	robotDrive = new RobotDrive(leftFrontMotor,
	                            leftRearMotor,
	                            rightFrontMotor,
	                            rightRearMotor);

	robotDrive->SetInvertedMotor(RobotDrive::kFrontLeftMotor, true);
	robotDrive->SetInvertedMotor(RobotDrive::kRearLeftMotor, true);

	robotDrive->IsSafetyEnabled();
}

//}
