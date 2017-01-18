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

void CrevoRobot::robotInit(void){

	/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

	//---------------Drive MotorControllers-----------------------

	//--Left Side--
	leftFrontMotor 	= new CANTalon(MotorCAN::LEFT_FRONT_PORT);
	leftRearMotor   = new CANTalon(MotorCAN::LEFT_REAR_PORT);
	/*
	 * Setting the Rear Motor as a Slave to the front motor, only needs to turn on one side)
	 */
	leftRearMotor->SetTalonControlMode(CANTalon::TalonControlMode::kFollowerMode);
	leftRearMotor->Set(leftFrontMotor->GetDeviceID());

	//--Right Side--
	rightFrontMotor = new CANTalon(MotorCAN::RIGHT_FRONT_PORT);
	rightRearMotor  = new CANTalon(MotorCAN::RIGHT_REAR_PORT);
	/*
	 * Setting the Rear Motor as a Slave to the front motor, only needs to turn on one side)
	 */
	rightRearMotor->SetTalonControlMode(CANTalon::TalonControlMode::kFollowerMode);
	rightRearMotor->Set(rightFrontMotor->GetDeviceID());

	//------------------------------------------------------------

	/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/


	//---------------Manipulators MotorControllers----------------
	intakeRoller    = new CANTalon(MotorCAN::INTAKE_MOTOR);
	fuelManipulator = new CANTalon(MotorCAN::SHOOTER_MOTOR);


	/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

	//---------------Configure MotorControlers--------------------

	//--Set Invert--
	intakeRoller->SetInverted(false);
	fuelManipulator->SetInverted(false);


	/* CTRE Magnetic Encoder
	 * Absolute Mode							Relative Mode
	 * Update rate: 4ms							Update rate: 100us
	 * Max RPM: 7,500RPM						Max RPM: 15,000RPM
	 * Accuracy: 12 bits/rotation (4096)		Accuracy: 12 bits/rotation
	 * API: Pulse Width API						USes Quadrature API
	 */

	fuelManipulator->SetFeedbackDevice(CANTalon::FeedbackDevice::CtreMagEncoder_Relative);
	leftFrontMotor->SetFeedbackDevice(CANTalon::FeedbackDevice::CtreMagEncoder_Absolute);
	rightFrontMotor->SetFeedbackDevice(CANTalon::FeedbackDevice::CtreMagEncoder_Absolute);

	//--
	leftFrontMotor->SetP(0.3);
	leftFrontMotor->SetI(0.5);

	/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

	//-------------------RobotDrive-------------------------------
	robotDrive = new RobotDrive(leftFrontMotor, rightFrontMotor);

	robotDrive->SetInvertedMotor(RobotDrive::kFrontLeftMotor, false);

	//robotDrive->SetInvertedMotor(RobotDrive::kRearLeftMotor, false);

	robotDrive->SetInvertedMotor(RobotDrive::kFrontRightMotor, false);

	//robotDrive->SetInvertedMotor(RobotDrive::kRearRightMotor, false);

	/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

	//--------------Configure Sensors-----------------------------
	gyro = new AnalogGyro(AnalogPort::GYRO);
	accel = new AnalogAccelerometer(AnalogPort::ACCELEROMETER);

	fuelManipulatorEncoder = new Encoder(DigitalPort::FUEL_MANIPULATOR_ENCODER_1, DigitalPort::FUEL_MANIPULATOR_ENCODER_2);

	leftEnc = new Encoder(DigitalPort::L_EN_1, DigitalPort::L_EN_2, true, Encoder::EncodingType::k2X);
	rightEnc = new Encoder(DigitalPort::R_EN_1, DigitalPort::R_EN_2, false, Encoder::EncodingType::k2X);

	leftEnc->SetSamplesToAverage(5);
	rightEnc->SetSamplesToAverage(5);

	//--Setting the distance for the encoders connected to the roborios IO port. --
	//leftEnc->SetDistancePerPulse(calcdistanceperPulse);
	//rightEnc->SetDistancePerPulse(calcdistanceperPulse);

	/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/



}

//}
