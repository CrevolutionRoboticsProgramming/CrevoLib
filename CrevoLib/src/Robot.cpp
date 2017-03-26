#include <iostream>
#include <memory>
#include <string>
#include <stdio.h>
#include <unistd.h>

#include <Commands/Command.h>
#include <IterativeRobot.h>
#include <Joystick.h>
#include <Timer.h>
#include <Preferences.h>
#include <networktables/NetworkTable.h>

#include <crevoglb.h>
#include <DriveTrain.h>
#include <AutonVectors.h>

#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>

class Robot: public IterativeRobot, public OI, public AutonVectors, public DriveTrain
{
public:

/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
	int auton;
	void RobotInit() override {

		driverGamepad      = new Joystick(0);
		operatorGamepad    = new Joystick(1);
		runTime            = new Timer();
		fdbk.elapsedRumble = new Timer();
		agitatorTimer      = new Timer();


		crvbot.robotInit();

		/*/
		 * Calibrates Gyro, needs two seconds in order to calibrate correctly.
		/*/
		crvbot.gyro->Calibrate();

		Wait(2);

		/*/
		 *	Command to start up the stream from the USB camera.
		/*/

		vs.startStream();

		AllianceLED();

		/*/
		 *  This is setting up the network table to communicate preferences from smart dashboard to the RoboRIO
		/*/
		prefs = Preferences::GetInstance();

		std::cout << "_____________________________________________" << std::endl;
		std::cout << "" << std::endl;
		std::cout << "|| Crevobot || Robot completed initialize"     << std::endl;
		std::cout << "_____________________________________________" << std::endl;
	}

	/*/
	 * 1. Shoot from hopper
	 * 2. Score gear right shoot from lift
	 * 3. Score gear center, shoot from base line
	 * 4. Score gear left, shooter from baseline
	 * 5. Score gear left, knock down hoppers
	 * 6. Hopper knock down
	 /*/

	void AutonomousInit() override {

//		/*/
//		 * Initializes the robots settings into the DriveTrain class to use its functions.
//		/*/
		crvbot.leftEnc->Reset();
		crvbot.rightEnc->Reset();

		crvbot.gyro->Reset();

		/*/
		 *  Init Shooter as controlled by velocity
		/*/

		//initDrive(crvbot.robotDrive);

		UpdateRobotStatus();
		UpdateRobotPreference();

		runTime->Reset();

		std::cout << "_____________________________________________" << std::endl;
		std::cout << "" << std::endl;
		std::cout << "|| Crevobot || In Autonomous Periodic Mode" << std::endl;
		std::cout << "_____________________________________________" << std::endl;
		HasReached = false;
	}
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
	bool HasReached;
	int autonSelect = 1;
	int boilerPostition;

	void AutonomousPeriodic() {

		runTime->Stop();
}
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
	double kP;
	double kI;
	double kD;
	double kF;

	void TeleopInit() {

		UpdateRobotStatus();
		UpdateRobotPreference();

		runTime->Reset();

		crvbot.gyro->Reset();

		crvbot.leftEnc->Reset();
		crvbot.rightEnc->Reset();

		crvbot.fuelShooterMaster->SelectProfileSlot(0);
		crvbot.fuelShooterMaster->SetP(kP);
		crvbot.fuelShooterMaster->SetI(kI);
		crvbot.fuelShooterMaster->SetD(kD);
		crvbot.fuelShooterMaster->SetF(kF);

		std::cout << "_____________________________________________" << std::endl;
		std::cout << "" << std::endl;
		std::cout << "|| Crevobot || In TeleopPeriodic Mode" << std::endl;
		std::cout << "_____________________________________________" << std::endl;
	}
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
	void TeleopPeriodic() {

		runTime->Start();

		while(IsOperatorControl() && IsEnabled())
		{
			/*_____ DRIVETRIAN CODE_____*/
			DriveCode();

			/*_____ SHOOTER CODE_____*/
			ShootingProcesses();

			/*_____AGITATOR CODE_____*/
			AgitatorCode();

			/*_____INTAKE CODE_____*/
			whilePressedAction(controllerButton(driverGamepad, Button::RightBumber), driverGamepad->GetRawAxis(2) > 0, crvbot.intakeRoller, 0.6);

			/*_____HANGER CODE_____*/
			HangerCode();

			/*_____ALIGNMENT CODE_____*/
			//AlineCheck();

			/*_____UPDATING ROBOT STATUS_____*/
			UpdateRobotStatus();
		}

		runTime->Stop();

	}

/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

	void TestPeriodic() {
		lw->Run();
	}
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

private:

	LiveWindow* lw = LiveWindow::GetInstance();

	CrevoRobot crvbot;
	Vision vs;
	FeedBack fdbk;
	Joystick *driverGamepad;
	Joystick *operatorGamepad;
	Preferences *prefs;
	Timer *runTime;
	Timer *agitatorTimer;

	bool ReverseDirection = false;
	bool HighSpeed = true;

	double TRN_SENIVITY;

	void DriveCode(void) {

	     double Left_Y  = controllerJoystick(driverGamepad, Axes::LEFT_Y);
	     double Right_Y = controllerJoystick(driverGamepad, Axes::RIGHT_Y);
		 double Right_X = TRN_SENIVITY * controllerJoystick(driverGamepad, Axes::RIGHT_X);

		 if(controllerButton(driverGamepad, Button::A))  	  ReverseDirection = true;
		 else if(controllerButton(driverGamepad, Button::B))  ReverseDirection = false;

		 if(controllerButton(driverGamepad, Button::LeftBumber)) { HighSpeed = false;}
		 else 													 { HighSpeed = true; }

		 if(!ReverseDirection)
			 if(HighSpeed)
				 crvbot.robotDrive->SetLeftRightMotorOutputs((LEFT_MULTIPLER*(Left_Y - Right_X)), (RIGHT_MULTIPLER*(Left_Y + Right_X)));
			 else
				 crvbot.robotDrive->SetLeftRightMotorOutputs(0.5 * (LEFT_MULTIPLER*(Left_Y - Right_X)), 0.5 * (RIGHT_MULTIPLER*(Left_Y + Right_X)));
	     else
	    	 if(HighSpeed)
	    		 crvbot.robotDrive->SetLeftRightMotorOutputs((LEFT_MULTIPLER*(-Left_Y - Right_X)), (RIGHT_MULTIPLER*(-Left_Y + Right_X)));
	    	 else
	    		 crvbot.robotDrive->SetLeftRightMotorOutputs(0.5 * (LEFT_MULTIPLER*(-Left_Y - Right_X)), 0.5 * (RIGHT_MULTIPLER*(-Left_Y + Right_X)));


	}

	double hangerSpeed;

	void HangerCode(void) {
		whilePressedAction(controllerButton(operatorGamepad, Button::A), controllerButton(operatorGamepad, Button::Start), crvbot.hangerMotor, hangerSpeed);
	}

	double currentRPM;
	double setRPM;
	double agitatorspeed;
	double Error;
	double reverseTime = 0.5;
	double IntergralError = 0;

	bool AgitatorEnabled = false;
	bool ReachedRPM;

	void ShootingProcesses(void) {
		currentRPM = crvbot.fuelShooterMaster->Get();
	/*
 	 *  Homemade PID control will add if Talon SRX PID fail
 	 *
		//Shooter is at negative RPM
		Error = setRPM  + currentRPM;
		IntergralError += Error;

		if((currentRPM > setRPM*.8) && (currentRPM < setRPM*1.2)){
			shooterMotorsSet( -(kI*IntergralError -kP*Error + setRPM));
		}
		else{

		*/
		if(operatorGamepad->GetRawAxis(3) > 0) {
		//Switches the Talon SRXs to Velocity control for PID
			crvbot.fuelShooterMaster->SetControlMode(CANSpeedController::kSpeed);
			crvbot.fuelShooterMaster->Set(-setRPM);

		}
		else if(operatorGamepad->GetRawAxis(2) > 0) {
		//Switches the Talon SRXs to Voltage Percentage control
			crvbot.fuelShooterMaster->SetControlMode(CANSpeedController::kPercentVbus);

			crvbot.fuelShooterMaster->Set(0.5);
		}
		else {
		//Switches the Talon SRXs to Voltage Percentage control
			crvbot.fuelShooterMaster->SetControlMode(CANSpeedController::kPercentVbus);
			crvbot.fuelShooterMaster->Set(0);
		 }




	}

	bool Completed = false;

	void AgitatorCode(void) {
		//AgitatorEnabled =  (crvbot.fuelShooterMaster->Get() >= (setRPM - 100));
/*
		if(AgitatorEnabled){
			ReachedRPM = true;
			crvbot.agitatorMotor->Set(agitatorspeed);
		}
		else {
		*/
			whilePressedAction(controllerButton(operatorGamepad, Button::RightBumber),
							   controllerButton(operatorGamepad, Button::LeftBumber),
							   crvbot.agitatorMotor, agitatorspeed);
			/*
		}
	*/

		/*
		 * AGITATOR SPINS FOR 0.28 SECONDS EVERY .52 SECONDS
		 * AGITATOR PULSE
		if(controllerButton(operatorGamepad, Button::RightBumber)) {

			agitatorTimer->Start();

			if(agitatorTimer->Get() < .28) {
				crvbot.agitatorMotor->Set(agitatorspeed);
			}
			else {
				if(agitatorTimer->Get() >= .28 && agitatorTimer->Get() < .9) {
					crvbot.agitatorMotor->StopMotor();
				}
				else {
					agitatorTimer->Reset();
				}
			}

		}
		else if(controllerButton(operatorGamepad, Button::LeftBumber)) {
			crvbot.agitatorMotor->Set(-agitatorspeed);
		}
		else {
			crvbot.agitatorMotor->StopMotor();
			agitatorTimer->Reset();
			Completed =false;
		}
		*/

	}

	void UpdateRobotStatus(void) {

		SmartDashboard::PutNumber(" Total Runtime: ",        		  runTime->Get());
		SmartDashboard::PutNumber(" LeftMotor Current: ",   		  crvbot.leftFrontMotor->GetOutputCurrent());
		SmartDashboard::PutNumber(" RightMotor Current: ",  		  crvbot.rightFrontMotor->GetOutputCurrent());
		SmartDashboard::PutNumber(" LeftMotor Voltage: ",   		  crvbot.leftFrontMotor->GetOutputVoltage());
		SmartDashboard::PutNumber(" RightMotor Voltage: ",  		  crvbot.rightFrontMotor->GetOutputVoltage());
		SmartDashboard::PutNumber(" fuelShooter Voltage: ", 	      crvbot.fuelShooterMaster->GetOutputVoltage());
		SmartDashboard::PutNumber(" fuelShooter Current: ",  	      crvbot.fuelShooterMaster->GetOutputCurrent());
		SmartDashboard::PutNumber(" Agitator Voltage: ",              crvbot.agitatorMotor->GetOutputVoltage());
		SmartDashboard::PutNumber(" Agitator Current: ",              crvbot.agitatorMotor->GetOutputCurrent());
		SmartDashboard::PutNumber(" Agitator Current: ",              crvbot.agitatorMotor->GetOutputCurrent());
		SmartDashboard::PutNumber(" Left Side Encoder Count: ", 	  crvbot.leftEnc->GetRaw());
		SmartDashboard::PutNumber(" Right Side Encoder Count: ",      crvbot.rightEnc->GetRaw());
		SmartDashboard::PutNumber(" fuelShooter Shooter Speed",       crvbot.fuelShooterMaster->Get());
		SmartDashboard::PutNumber(" fuelShooter encoder Postion", 	  crvbot.fuelShooterMaster->GetEncPosition());
		SmartDashboard::PutNumber(" fuelShooter RPM: ",      		  currentRPM);
		SmartDashboard::PutNumber(" fuelShooter RPM Graph: ",      	  crvbot.fuelShooterMaster->Get());
		SmartDashboard::PutNumber(" fuelShooter Error:",              Error);
		SmartDashboard::PutNumber(" Gyro Angle : ", 				  crvbot.gyro->GetAngle());
		SmartDashboard::PutNumber(" Alignment : ",					  boilerPosition);
		SmartDashboard::PutBoolean(" ReverseDirection ", 			  ReverseDirection);
		SmartDashboard::PutNumber(" Agitator Timer :",                agitatorTimer->Get());
		SmartDashboard::PutBoolean(" Align to boiler ",               BoilerInRange);
		SmartDashboard::PutBoolean(" Reached Set RPM ",               ReachedRPM);
		SmartDashboard::PutBoolean(" Agitator Enabled ",              AgitatorEnabled);
	}

	void UpdateRobotPreference(void) {

		leftMost 		  = prefs->GetInt("leftMost",     0);
		rightMost 		  = prefs->GetInt("rightMost",    500);
		allianceSide      = prefs->GetInt("Alliance Side", 0);
		setRPM		      = prefs->GetDouble("shooterspeed", .75);
		hangerSpeed       = prefs->GetDouble("Hanger Motor Speed", 0.75);
		agitatorspeed     = prefs->GetDouble("agitatorspeed", 0.25);
		kP	    		  = prefs->GetDouble("P", 0.1);
		kI	      		  = prefs->GetDouble("I", 0.00000001);
		kD	 			  = prefs->GetDouble("D", 0.0);
		kF				  = prefs->GetDouble("F", 0.04);
		TRN_SENIVITY      = prefs->GetDouble("Turn Sensitivity", 0.4);
	}

	int allianceSide;

	void AllianceLED(void){

		if(allianceSide == AllianceSide::BLUE) { crvbot.BlueLED->Set(true); }
		if(allianceSide == AllianceSide::RED)  { crvbot.RedLED->Set(true); }
	}

	int boilerPosition;
	int leftMost;
	int rightMost;

	bool BoilerInRange = false;

	void AlineCheck(void)
	{
		if((operatorGamepad->GetRawAxis(3) > 0) && (operatorGamepad->GetRawAxis(3) <= 0.5))
		{
			boilerPosition = vs.alignmentToBoiler();
			if(boilerPosition <= 0) BoilerInRange = false;
			else                    BoilerInRange = true;
		}
	}
};

START_ROBOT_CLASS(Robot)
