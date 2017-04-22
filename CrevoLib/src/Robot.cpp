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
#include <TalonSRX.h>

#include "crevoglb.h"
#include "DriveTrain.h"
#include "AutonVectors.h"

#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>

class Robot: public IterativeRobot, public OI, public DriveTrain
{
public:

/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

	void RobotInit() override {

		SmartDashboard::PutString(CrevoRobotState, "Robot initializing");
		SmartDashboard::PutString("Auton State : ", "NULL");
		SmartDashboard::PutString(av.SmartDashboardAutonKey, "NULL");

		driverGamepad      = new Joystick(0);
		operatorGamepad    = new Joystick(1);
		runTime            = new Timer();
		agitatorTimer      = new Timer();
		shooterTimer   	   = new Timer();

		//Initilizes all Talon SRX, Sensors, and joyticks.
		crvbot.robotInit();

		//Calibrates Gyro, needs two seconds in order to calibrate correctly.
		crvbot.gyro->Calibrate();

		/*/
		 *	Command to start up the stream from the USB camera.
		/*/

		vs.startStream();
		/*/
		 *  This is setting up the network table to communicate preferences from smart dashboard to the RoboRIO
		/*/
		prefs = Preferences::GetInstance();

		av.selectedAllianceSide.AddObject(av.redAlliance, av.redAlliance);
		av.selectedAllianceSide.AddObject(av.blueAlliance, av.blueAlliance);
		av.selectedAllianceSide.AddDefault("Pls Choose Alliance", "None Selected");
		frc::SmartDashboard::PutData("Alliance Side", &av.selectedAllianceSide);

		av.chooser.AddObject(av.GearCenter, av.GearCenter);
		av.chooser.AddObject(av.GearLeft,av. GearLeft);
		av.chooser.AddObject(av.GearRight, av.GearRight);
		av.chooser.AddObject( av.ShootFromWallRed, av.ShootFromWallRed);
		av.chooser.AddObject( av.ShootFromWallBlue, av.ShootFromWallBlue);
		av.chooser.AddObject(av.GearCenterTimed, av.GearCenterTimed);
		av.chooser.AddObject(av.Baseline, av.Baseline);
		av.chooser.AddDefault(av.idle, av.idle);
		frc::SmartDashboard::PutData("Auton List", &av.chooser);

		std::cout << "|| Crevobot || Robot completed initialize"     << std::endl;

		SmartDashboard::PutString(CrevoRobotState, "Robot completed initialization");

	}

	/*/
	 * 1. Shoot from hopper
	 * 2. Score gear right shoot from lift
	 * 3. Score gear center, shoot from base line
	 * 4. Score gear left, shooter from baseline
	 * 5. Score gear left, knock down hoppers
	 * 6. Hopper knock down
	 /*/

	int allianceSide;

	void AutonomousInit() override {

		av.AutonSelect(); 						//Lets pull in values from the smartdashboard chooser to choose auton

		UpdateRobotStatus();					//Update robot values so we can see any changes that has happened
		UpdateRobotChooser();


		crvbot.fuelShooterMaster->SetP(.5);  	//Sets all PID values for auton
		crvbot.fuelShooterMaster->SetI(.001);
		crvbot.fuelShooterMaster->SetD(25);


		runTime->Reset();           			//Reset timer for new time


		crvbot.gyro->Reset();					//Reset all sensors for new run
		crvbot.leftEnc->Reset();
		crvbot.rightEnc->Reset();

		init(crvbot.robotDrive, crvbot.gyro, EncoderType::kQuadEncoder, SelectedEncoder::kLeft);  //Now lets init our robot configuratoin to
		initEncoder(crvbot.leftEnc, crvbot.rightEnc);								  			  //our Drivetrain class

		std::cout << "|| Crevobot || In Autonomous Periodic Mode" << std::endl;

		SmartDashboard::PutString(CrevoRobotState, "In Auton Periodic");          //Put in what state the robot is in
		SmartDashboard::PutString(av.SmartDashboardAutonKey, "Auton Initilized");



	}
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
	int boilerPostition;

    int autonSelect;

	void AutonomousPeriodic() {

		runTime->Start();

		while(IsAutonomous() && IsEnabled()) {
			av.AutonStateProcess();
		}

			runTime->Stop();
	}

/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
	double kP;
	double kI;
	double kD;
	double kF;

	void TeleopInit() {

		UpdateRobotStatus();
		UpdateRobotChooser();

		runTime->Reset();

		//Reset all sensors for new run
		crvbot.gyro->Reset();
		crvbot.leftEnc->Reset();
		crvbot.rightEnc->Reset();


		crvbot.fuelShooterMaster->SelectProfileSlot(0);
		crvbot.fuelShooterMaster->SetPID(kP, kI, kD, kF); //Now lets put those PID values for that Talon Shooter

		std::cout << "|| Crevobot || In TeleopPeriodic Mode" << std::endl; // Now lets update so we can see the robot status
		SmartDashboard::PutString(CrevoRobotState, "In Teleop Periodic");

	}
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
	void TeleopPeriodic() {

		while(IsOperatorControl() && IsEnabled())
		{
			/*_____ DRIVETRIAN CODE_____*/
			DriveCode();

			/*_____ SHOOTER CODE_____*/
			ShootingProcesses();

			/*_____AGITATOR CODE_____*/
			AgitatorCode();

			/*_____INTAKE CODE_____*/
			IntakeCode();

			/*_____HANGER CODE_____*/
			HangerCode();

			/*_____ALIGNMENT CODE_____*/
			AlignCheck();

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

	AutonVectors av;
	CrevoRobot crvbot;
	Vision vs;
	Joystick *driverGamepad;
	Joystick *operatorGamepad;
	Preferences *prefs;
	Timer *runTime;
	Timer *agitatorTimer;
	Timer *shooterTimer;

	std::string  CrevoRobotState = "Crevobot State : ";

	bool ReverseDirection = false;
	bool HighSpeed = true;

	double TRN_SENIVITY;

	void DriveCode(void) {

	     double Left_Y  = controllerJoystick(driverGamepad, Axes::LEFT_Y);					 //Getting values from Driver game-pad joysticks
		 double Right_X = TRN_SENIVITY * controllerJoystick(driverGamepad, Axes::RIGHT_X);

		 if(controllerButton(driverGamepad, Button::A))  	  	  ReverseDirection = true;   //Logic to change the drivetrain's direction
		 else if(controllerButton(driverGamepad, Button::B))  	  ReverseDirection = false;

		 if(controllerButton(driverGamepad, Button::LeftBumber))  HighSpeed = false; 		 //Logic to change the drivetrain's speed
		 else 													  HighSpeed = true;

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

	double currentRPM;
	double setRPM;
	double agitatorspeed;
	double Error;
	double reverseTime = 0.5;
	double IntergralError = 0;

	bool AgitatorEnabled = false;
	bool ReachedRPM;

	void ShootingProcesses(void) {

		//Activates Talon SRX PID control to set velcocity
		if(operatorGamepad->GetRawAxis(3) > 0) {
		//Switches the Talon SRXs to Velocity control for PID
			crvbot.fuelShooterMaster->SetControlMode(CANSpeedController::kSpeed);
			crvbot.fuelShooterMaster->Set(-setRPM);

		}
		//Reverse option if shooter gets jammed
		else if(operatorGamepad->GetRawAxis(2) > 0) {
		//Switches the Talon SRXs to Voltage Percentage control
			crvbot.fuelShooterMaster->SetControlMode(CANSpeedController::kPercentVbus);

			crvbot.fuelShooterMaster->Set(0.75);
		}
		//No button action, no movement
		else {
		//Switches the Talon SRXs to Voltage Percentage control
			crvbot.fuelShooterMaster->SetControlMode(CANSpeedController::kPercentVbus);
			crvbot.fuelShooterMaster->Set(0);
		 }

		currentRPM = crvbot.fuelShooterMaster->Get();

		//Logic to tell operator if shooter reachd the given RPM
		if(currentRPM >= (setRPM - 100)) ReachedRPM = true;
		else 							 ReachedRPM = false;

	}

	void AgitatorCode(void) {
		whilePressedAction(controllerButton(operatorGamepad, Button::RightBumber), controllerButton(operatorGamepad, Button::LeftBumber), crvbot.agitatorMotor, agitatorspeed);
	}

	void IntakeCode(void) {
		whilePressedAction(controllerButton(driverGamepad, Button::RightBumber), driverGamepad->GetRawAxis(2) > 0, crvbot.intakeRoller, 0.6);
	}


	void HangerCode(void) {
		whilePressedAction(controllerButton(operatorGamepad, Button::A), controllerButton(operatorGamepad, Button::Start), crvbot.hangerMotor, 1);
	}

	int boilerPosition;
	int leftMost;
	int rightMost;

	bool BoilerInRange;
	bool VisionTracking;

	void AlignCheck(void) {
		// Checks to see if operator selected to have vision tracking enabled
		if(VisionTracking){

			// Only runs if the operator pulls down the trigger a bit to aline robot
			if((operatorGamepad->GetRawAxis(3) > 0) && (operatorGamepad->GetRawAxis(3) <= 0.5))
			{
				// Gets X value placement of teh boiler
				boilerPosition = vs.alignmentToBoiler();
				//Simple logic to tell if boiler is alingned or not
				if(boilerPosition <= 0) BoilerInRange = false;
				else                    BoilerInRange = true;
			}
		}
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
		SmartDashboard::PutNumber(" Agitator Timer :",                agitatorTimer->Get());
		SmartDashboard::PutBoolean(" ReverseDirection ", 			  ReverseDirection);
		SmartDashboard::PutBoolean(" Align to boiler ",               BoilerInRange);
		SmartDashboard::PutBoolean(" Reached Set RPM ",               ReachedRPM);
		SmartDashboard::PutBoolean(" Agitator Enabled ",              AgitatorEnabled);
	}

	void UpdateRobotChooser(void) {

		leftMost 		  = prefs->GetInt("leftMost",     0);
		rightMost 		  = prefs->GetInt("rightMost",    500);
		allianceSide      = prefs->GetInt("Alliance Side", 0);
		setRPM		      = prefs->GetDouble("shooterspeed", .75);
		agitatorspeed     = prefs->GetDouble("agitatorspeed", 0.25);
		kP	    		  = prefs->GetDouble("P", 0.1);
		kI	      		  = prefs->GetDouble("I", 0.00000001);
		kD	 			  = prefs->GetDouble("D", 0.0);
		kF				  = prefs->GetDouble("F", 0.04);
		TRN_SENIVITY      = prefs->GetDouble("Turn Sensitivity", 0.65);
		VisionTracking    = prefs->GetBoolean("Vision Tracking", false);

		av.autonSelected = av.chooser.GetSelected();


	}

};

START_ROBOT_CLASS(Robot)
