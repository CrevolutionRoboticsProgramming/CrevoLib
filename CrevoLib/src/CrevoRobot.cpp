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

	/*________________________________________________________________________________________________________________________________*/

	//___________________ Drive MotorControllers ___________________

	//--Left Side--
    leftFrontMotor  = new CANTalon(MotorCAN::LEFT_FRONT_PORT);
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

	//________________________________________________________________

	/*________________________________________________________________________________________________________________________________*/


	//___________________ Manipulators MotorControllers ___________________
	intakeRoller    = new CANTalon(MotorCAN::INTAKE_MOTOR);
	fuelManipulator = new CANTalon(MotorCAN::SHOOTER_MOTOR);


	/*________________________________________________________________________________________________________________________________*/

	//___________________ Configure MotorControlers ___________________

	//--Set Invert--
	if(intakeRoller    != NULL)    intakeRoller->SetInverted(false);
	if(fuelManipulator != NULL) fuelManipulator->SetInverted(false);


	/* CTRE Magnetic Encoder
	 * Absolute Mode							Relative Mode
	 * Update rate: 4ms							Update rate: 100us
	 * Max RPM: 7,500RPM						Max RPM: 15,000RPM
	 * Accuracy: 12 bits/rotation (4096)		Accuracy: 12 bits/rotation
	 * API: Pulse Width API						USes Quadrature API
	 */

	if(leftFrontMotor  != NULL)  leftFrontMotor->SetFeedbackDevice(CANTalon::FeedbackDevice::CtreMagEncoder_Absolute);
	if(rightFrontMotor != NULL) rightFrontMotor->SetFeedbackDevice(CANTalon::FeedbackDevice::CtreMagEncoder_Absolute);

	//--
	//leftFrontMotor->SetP(0.3);
	//leftFrontMotor->SetI(0.5);

	if(leftFrontMotor  != NULL)  leftFrontMotor->Set(0);
	if(rightFrontMotor != NULL) rightFrontMotor->Set(0);
	if(intakeRoller    != NULL)	   intakeRoller->Set(0);
	if(fuelManipulator != NULL) fuelManipulator->Set(0);

	//_____________________________ PID Configuration _____________________________

	if(fuelManipulator != NULL) fuelManipulator->SetFeedbackDevice(CANTalon::FeedbackDevice::CtreMagEncoder_Relative);

	// These values are for PID control. Will be Adjusted later.

	fuelManipulator->SetSensorDirection(false);
	fuelManipulator->ConfigNominalOutputVoltage(0.0f, -0.0f);
	fuelManipulator->ConfigPeakOutputVoltage(12.0f, 0.0f);

	fuelManipulator->SetPID(0,0,0);




	/*________________________________________________________________________________________________________________________________*/

	//______________________________ RobotDrive ______________________________________
	robotDrive = new RobotDrive(leftFrontMotor, rightFrontMotor);

	robotDrive->SetInvertedMotor(RobotDrive::kFrontLeftMotor, false);

	//robotDrive->SetInvertedMotor(RobotDrive::kRearLeftMotor, false);

	robotDrive->SetInvertedMotor(RobotDrive::kFrontRightMotor, false);

	//robotDrive->SetInvertedMotor(RobotDrive::kRearRightMotor, false);

	/*________________________________________________________________________________________________________________________________*/

	//____________________________ Configure Sensors _____________________________________
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

	/*________________________________________________________________________________________________________________________________*/



}

//}
