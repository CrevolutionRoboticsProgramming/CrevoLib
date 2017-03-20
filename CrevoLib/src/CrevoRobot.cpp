/*
 * CrevoRobot.cpp
 *
 *  Created on: Dec 19, 2016
 *      Author: Martin
 */

#include <CrevoRobot.h>
#include <SmartDashboard/SmartDashboard.h>

CrevoRobot::CrevoRobot(){

}

CrevoRobot::~CrevoRobot(){

}

void CrevoRobot::robotInit(void){

	/*________________________________________________________________________________________________________________________________*/
#ifdef NOT_DEBUG
	//___________________ Drive MotorControllers ___________________


	//--Left Side--
    leftFrontMotor   = new CANTalon(MotorCAN::LEFT_FRONT_PORT);

	leftRearMotor    = new CANTalon(MotorCAN::LEFT_REAR_PORT);

	//--Right Side--
	rightFrontMotor  = new CANTalon(MotorCAN::RIGHT_FRONT_PORT);

	rightRearMotor   = new CANTalon(MotorCAN::RIGHT_REAR_PORT);

	//___________________ Manipulators MotorControllers ___________________

	fuelShooterMaster      = new CANTalon(MotorCAN::SHOOTER_MOTOR_A);

	fuelShooterSlave     = new CANTalon(MotorCAN::SHOOTER_MOTOR_B);

	intakeRoller   	 = new CANTalon(MotorCAN::INTAKE_MOTOR);

	agitatorMotor	 = new CANTalon(MotorCAN::AGITATOR_MOTOR);

	hangerMotor		 = new CANTalon(MotorCAN::HANGER_MOTOR);

	/*________________________________________________________________________________________________________________________________*/

	SmartDashboard::PutBoolean(" Debug: ", false);

	SmartDashboard::PutNumber("Left Front CANTalon ID: ",    (int)leftFrontMotor->GetDeviceID());
	SmartDashboard::PutNumber("Left Rear CANTalon ID: ",     (int)leftRearMotor->GetDeviceID());
	SmartDashboard::PutNumber("Right Front CANTalon ID: ",   (int)rightFrontMotor->GetDeviceID());
	SmartDashboard::PutNumber("Right Rear CANTalon ID: ",    (int)rightRearMotor->GetDeviceID());
	SmartDashboard::PutNumber("Fuel Shooter CANTalon1 ID: ", (int)fuelShooterMaster->GetDeviceID());
	SmartDashboard::PutNumber("Fuel Shooter CANTalon2 ID: ", (int)fuelShooterSlave->GetDeviceID());
	SmartDashboard::PutNumber("In-take CANTalon ID: ",       (int)intakeRoller->GetDeviceID());
	SmartDashboard::PutNumber("Agitator CANTalon ID: ",      (int)agitatorMotor->GetDeviceID());
	SmartDashboard::PutNumber("Hanger CANTalon ID: ", 		 (int)hangerMotor->GetDeviceID());
#endif

#ifndef NOT_DEBUG
	SmartDashboard::PutBoolean(" Debug: ", true);
#endif
	/*________________________________________________________________________________________________________________________________*/

	//___________________ Configure MotorControlers ___________________

	/*
	* Setting the Rear Motor as a Slave to the front motor, only needs to turn on one side)
	*/
	leftRearMotor->SetTalonControlMode(CANTalon::TalonControlMode::kFollowerMode);
	leftRearMotor->Set(leftFrontMotor->GetDeviceID());

	/*
	 * Setting the Rear Motor as a Slave to the front motor, only needs to turn on one side)
	 */
	rightRearMotor->SetTalonControlMode(CANTalon::TalonControlMode::kFollowerMode);
	rightRearMotor->Set(rightFrontMotor->GetDeviceID());


	if(leftFrontMotor  != NULL) leftFrontMotor->SetInverted(true);
	if(leftFrontMotor  != NULL) leftFrontMotor->ConfigNominalOutputVoltage(0.0, -0.0);

	if(rightFrontMotor != NULL) rightFrontMotor->SetInverted(true);
	if(rightFrontMotor != NULL) rightFrontMotor->ConfigNominalOutputVoltage(0.0, -0.0);

	if(intakeRoller    != NULL) intakeRoller->SetInverted(false);
	if(intakeRoller	   != NULL) intakeRoller->SetVoltageRampRate(0.5);
	if(intakeRoller    != NULL) intakeRoller->ConfigNominalOutputVoltage(0.0, -0.0);

	if(hangerMotor     != NULL) hangerMotor->SetInverted(false);
	if(hangerMotor     != NULL) hangerMotor->ConfigNominalOutputVoltage(0.0, -0.0);

	/*/
	 * CTRE Magnetic Encoder
	 * Absolute Mode							Relative Mode
	 * Update rate: 4ms							Update rate: 100us
	 * Max RPM: 7,500RPM						Max RPM: 15,000RPM
	 * Accuracy: 12 bits/rotation (4096)		Accuracy: 12 bits/rotation
	 * API: Pulse Width API						pulse Quadrature API
	/*/

	//_____________________________ PID Configuration _____________________________


	// These values are for PID control. Will be Adjusted later.
	if(fuelShooterSlave    != NULL) fuelShooterSlave->SetTalonControlMode(CANTalon::TalonControlMode::kFollowerMode);
	if(fuelShooterSlave    != NULL) fuelShooterSlave->Set(fuelShooterMaster->GetDeviceID());

	if(fuelShooterMaster     != NULL) fuelShooterMaster->SetFeedbackDevice(CANTalon::CtreMagEncoder_Relative);
	if(fuelShooterMaster     != NULL) fuelShooterMaster->SetSensorDirection(false);

	if(fuelShooterMaster     != NULL) fuelShooterMaster->ConfigNominalOutputVoltage(0.0, -0.0);
	if(fuelShooterMaster     != NULL) fuelShooterMaster->ConfigPeakOutputVoltage(12, -12);

	//Taken out RampRate
	//if(fuelShooterMaster != NULL) fuelShooterMaster->SetVoltageRampRate(0.2);

	if(fuelShooterMaster     != NULL) fuelShooterMaster->SetInverted(true);

	/*________________________________________________________________________________________________________________________________*/

	//______________________________ RobotDrive ______________________________________

	robotDrive = new RobotDrive(leftFrontMotor, rightFrontMotor);

	/*________________________________________________________________________________________________________________________________*/

	//____________________________ Configure Sensors _____________________________________
	gyro       = new AnalogGyro(AnalogPort::GYRO);
	accel      = new AnalogAccelerometer(AnalogPort::ACCELEROMETER);

	gearSensor = new DigitalInput(DigitalPort::GEAR_SENSOR);

	leftEnc    = new Encoder(DigitalPort::L_EN_1, DigitalPort::L_EN_2, true, Encoder::EncodingType::k2X);
	rightEnc   = new Encoder(DigitalPort::R_EN_1, DigitalPort::R_EN_2, false, Encoder::EncodingType::k2X);

	//leftEnc->SetSamplesToAverage(5);
	//rightEnc->SetSamplesToAverage(5);

	//--Setting the distance for the encoders connected to the roboRIO's IO port. --
	//leftEnc->SetDistancePerPulse(calcdistanceperPulse);
	//rightEnc->SetDistancePerPulse(calcdistanceperPulse);

	/*________________________________________________________________________________________________________________________________*/

	/*_____ Set All motor Percentages to Zero _____*/
	if(robotDrive      != NULL)   robotDrive->StopMotor();
	if(fuelShooterMaster 	   != NULL)   fuelShooterMaster->Set(0);
	if(fuelShooterSlave    != NULL)   fuelShooterSlave->Set(0);
	if(intakeRoller    != NULL)   intakeRoller->Set(0);
	if(hangerMotor     != NULL)   hangerMotor->Set(0);

#ifdef ROBOT_1
	SmartDashboard::PutString("Robot Configuration: ", "ROBOT 1");
#endif

#ifndef ROBOT_1
	SmartDashboard::PutString("Robot Configuration: ", "ROBOT 2");
#endif
}

