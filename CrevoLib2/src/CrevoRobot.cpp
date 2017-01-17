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
/*
	//---------------Drive MotorControllers-----------------------
	leftFrontMotor = new CANTalon(MotorCAN::LEFT_FRONT_PORT);
	leftRearMotor = new CANTalon(MotorCAN::LEFT_REAR_PORT);
	rightFrontMotor = new CANTalon(MotorCAN::RIGHT_FRONT_PORT);
	rightRearMotor = new CANTalon(MotorCAN::RIGHT_REAR_PORT);
	//------------------------------------------------------------

	//---------------Manipulators MotorControllers----------------
	intakeRoller = new CANTalon(MotorCAN::INTAKE_MOTOR);
	fuelManipulator = new CANTalon(MotorCAN::SHOOTER_MOTOR);
	//------------------------------------------------------------

	//---------------Configure MotorControlers--------------------

	intakeRoller->SetInverted(false);
	fuelManipulator->SetInverted(false);

	leftFrontMotor->SetP(0.3);
	leftFrontMotor->SetI(0.5);

	//------------------------------------------------------------

	//-------------------RobotDrive-------------------------------
	robotDrive =  new RobotDrive(leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor);

	robotDrive->SetInvertedMotor(RobotDrive::kFrontLeftMotor, false);
	robotDrive->SetInvertedMotor(RobotDrive::kRearLeftMotor, false);
	robotDrive->SetInvertedMotor(RobotDrive::kFrontRightMotor, false);
	robotDrive->SetInvertedMotor(RobotDrive::kRearRightMotor, false);
	//------------------------------------------------------------


	//--------------Configure Sensors-----------------------------
	gyro = new AnalogGyro(AnalogPort::GYRO);
	accel = new AnalogAccelerometer(AnalogPort::ACCELEROMETER);

	fuelManipulatorEncoder = new Encoder(DigitalPort::FUEL_MANIPULATOR_ENCODER_1, DigitalPort::FUEL_MANIPULATOR_ENCODER_2);
	leftEnc = new Encoder(DigitalPort::L_EN_1, DigitalPort::L_EN_2, true);
	rightEnc = new Encoder(DigitalPort::R_EN_1, DigitalPort::R_EN_2, false);

	leftEnc->SetSamplesToAverage(5);
	rightEnc->SetSamplesToAverage(5);

	leftEnc->SetDistancePerPulse(calcdistanceperPulse);
	rightEnc->SetDistancePerPulse(calcdistanceperPulse);

	//------------------------------------------------------------

*/
}

//}
