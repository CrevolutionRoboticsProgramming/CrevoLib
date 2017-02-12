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
#ifdef NOT_DEBUG
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


	fuelManipulator  = new CANTalon(MotorCAN::SHOOTER_MOTOR_A);
	fuelManipulator2 = new CANTalon(MotorCAN::SHOOTER_MOTOR_B);

	intakeRoller   	 = new CANTalon(MotorCAN::INTAKE_MOTOR);

	SmartDashboard::PutBoolean(" Debug: ", false);

#endif

#ifndef NOT_DEBUG
	SmartDashboard::PutBoolean(" Debug: ", true);
#endif
	/*________________________________________________________________________________________________________________________________*/

	//___________________ Configure MotorControlers ___________________

	//--Set Invert--

	if(fuelManipulator != NULL) fuelManipulator2->SetTalonControlMode(CANTalon::TalonControlMode::kFollowerMode);
	if(fuelManipulator != NULL) fuelManipulator2->Set(fuelManipulator->GetDeviceID());

	if(leftFrontMotor  != NULL)  leftFrontMotor->SetInverted(true);
	if(rightFrontMotor != NULL) rightFrontMotor->SetInverted(true);
	if(fuelManipulator != NULL) fuelManipulator->SetInverted(false);
	if(intakeRoller    != NULL) intakeRoller->SetInverted(false);
	if(hangerMotor     != NULL) hangerMotor->SetInverted(false);

	/*/
	 * CTRE Magnetic Encoder
	 * Absolute Mode							Relative Mode
	 * Update rate: 4ms							Update rate: 100us
	 * Max RPM: 7,500RPM						Max RPM: 15,000RPM
	 * Accuracy: 12 bits/rotation (4096)		Accuracy: 12 bits/rotation
	 * API: Pulse Width API						USes Quadrature API
	/*/
/*
	if(leftFrontMotor  != NULL)  leftFrontMotor->SetFeedbackDevice(CANTalon::FeedbackDevice::CtreMagEncoder_Absolute);
	if(rightFrontMotor != NULL) rightFrontMotor->SetFeedbackDevice(CANTalon::FeedbackDevice::CtreMagEncoder_Absolute);
*/
	//--
	//leftFrontMotor->SetP(0.3);
	//leftFrontMotor->SetI(0.5);
	//_____________________________ PID Configuration _____________________________



	// These values are for PID control. Will be Adjusted later.

	if(fuelManipulator != NULL) fuelManipulator->ConfigNominalOutputVoltage(0.0f, -0.0f);
	if(fuelManipulator != NULL) fuelManipulator->ConfigPeakOutputVoltage(12.0f, 0.0f);

	if(fuelManipulator != NULL) fuelManipulator->SetPID(0,0,0);



	/*________________________________________________________________________________________________________________________________*/

	//______________________________ RobotDrive ______________________________________
	robotDrive = new RobotDrive(leftFrontMotor, rightFrontMotor);

	/*________________________________________________________________________________________________________________________________*/

	//____________________________ Configure Sensors _____________________________________
	gyro       = new AnalogGyro(AnalogPort::GYRO);
	accel      = new AnalogAccelerometer(AnalogPort::ACCELEROMETER);

	if(fuelManipulator != NULL) fuelManipulator->SetFeedbackDevice(CANTalon::FeedbackDevice::CtreMagEncoder_Relative);
	if(fuelManipulator != NULL) fuelManipulator->SetSensorDirection(false);

	leftEnc    = new Encoder(DigitalPort::L_EN_1, DigitalPort::L_EN_2, true, Encoder::EncodingType::k2X);
	rightEnc   = new Encoder(DigitalPort::R_EN_1, DigitalPort::R_EN_2, false, Encoder::EncodingType::k2X);

	//leftEnc->SetSamplesToAverage(5);
	//rightEnc->SetSamplesToAverage(5);

	//--Setting the distance for the encoders connected to the roboRIO's IO port. --
	//leftEnc->SetDistancePerPulse(calcdistanceperPulse);
	//rightEnc->SetDistancePerPulse(calcdistanceperPulse);

	/*________________________________________________________________________________________________________________________________*/

	table = NetworkTable::GetTable("GRIP/Crevo");

	/*_____ Set All motor Percentages to Zero _____*/
	if(robotDrive      != NULL)      robotDrive->StopMotor();
	if(fuelManipulator != NULL) fuelManipulator->StopMotor();
	if(intakeRoller    != NULL)    intakeRoller->StopMotor();
	if(hangerMotor     != NULL)     hangerMotor->StopMotor();
#ifdef PRAC_BOT
	SmartDashboard::PutString("Robot Configuration: ", "Practice Bot");
#endif

#ifndef PRAC_BOT
	SmartDashboard::PutString("Robot Configuration: ", "Competition Bot");
#endif
}

