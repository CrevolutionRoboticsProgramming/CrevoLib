#include <iostream>
#include <memory>
#include <string>
#include <stdio.h>
#include <unistd.h>

#include <Commands/Command.h>
#include <IterativeRobot.h>
#include <Joystick.h>
#include <Timer.h>
#include <PIDController.h>
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

		crvbot.robotInit();

		/*/
		 * Calibrates Gyro, needs two seconds in order to calibrate correctly.
		/*/
		crvbot.gyro->Calibrate();

		Wait(2);

		/*/
		 *	Command to start up the stream from the USB camera.
		/*/

		//vs.startStream();

		/*/
		 *  This is setting up the network table to communicate preferences from smart dashboard to the RoboRIO
		/*/
		prefs = Preferences::GetInstance();

		std::cout << "_____________________________________________" << std::endl;
		std::cout << "" << std::endl;
		std::cout << "|| Crevobot || Robot completed initialize" << std::endl;
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
		init(crvbot.robotDrive, crvbot.gyro, EncoderType::kQuadEncoder, SelectedEncoder::kLeft);


		updateRobotStatus();
		updateRobotPreference();

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
		runTime->Start();

		while(IsAutonomous() && IsEnabled())
		{
		switch(auton)
		{
		case 0:
		{


				//AutonSelect(AutonStratagey::SHOOT_FROM_HOPPER);

	//				while(vs.alinementToBoiler() <=  200  && !hasReached)
	//				{
	//					drvt.moveRobot(0.15, -0.15);
	//				}				hasReached = true;
	//				drvt.moveRobot(0,0);
				while(!HasReached){

					while(A1_M1_LeftCount  <= crvbot.leftEnc->GetRaw() ) {
							moveRobot(A1_M1_Speed);
							updateRobotStatus();
						 }
				HasReached = true;
				}
					break;
			}

		case 1:
		{
			while(A2_M1_LeftCount >= crvbot.leftEnc->GetRaw())
			{
				moveRobot(A2_M1_Speed);
				updateRobotStatus();
			}
			stopRobot();

			while(-A2_M2_GyroAngle < crvbot.gyro->GetAngle())
			{
				moveRobot(A2_M2_Speed, -A2_M2_Speed);
				updateRobotStatus();
			}

			stopRobot();
			Wait(1);
			crvbot.leftEnc->Reset();

			while(A2_M3_LeftCount >= crvbot.leftEnc->GetRaw() )
			{
				moveRobot(A2_M3_Speed);
				updateRobotStatus();
			}
			stopRobot();
			auton = 2851;
			break;
		}
		case 3:
		{

			while(crvbot.leftEnc->GetRaw() <= A3_M1_LeftCount)
			{
				moveRobot(0.3, 0.3);
				updateRobotStatus();
			}

			stopRobot();
			crvbot.gyro->Reset();
			while(crvbot.gyro->GetAngle() <= A3_M2_GyroAngle)
			{
				moveRobot(-0.3, 0.3);
				updateRobotStatus();
			}
			stopRobot();
			crvbot.leftEnc->Reset();
			Wait(0.5);
			while(crvbot.leftEnc->GetRaw() <= A3_M3_LeftCount)
			{
				moveRobot(0.3, 0.3);
			}

			stopRobot();
			auton = 2851;
			break;
		}
		case 4:
		{
			int placeholder;
			while(crvbot.leftEnc->GetRaw() <= placeholder)
			{
				moveRobot(0.3, 0.3);
				updateRobotStatus();
			}
			stopRobot();
			updateRobotStatus();

			while(crvbot.gyro->GetAngle() <= 90)
			{
				moveRobot(0.3, -0.3);
				updateRobotStatus();
			}
			stopAndReset();
			Wait(1);
			updateRobotStatus();

			while(crvbot.leftEnc->GetRaw() <= placeholder)
			{
				moveRobot(0.3, 0.3);
				updateRobotStatus();
			}
			stopRobot();
			Wait(3);
			updateRobotStatus();

			while(crvbot.leftEnc->GetRaw() >= 0)
			{
				moveRobot(-0.3, -0.3);
				updateRobotStatus();
			}
			stopAndReset();
			updateRobotStatus();

			while(crvbot.gyro->GetAngle() <= 45)
			{
				moveRobot(0.3, -0.3);
				updateRobotStatus();
			}
			updateRobotStatus();
			auton = 2851;
			break;
		}
		case 2851:
		{
			stopRobot();
			crvbot.fuelShooterMaster->StopMotor();
			break;
		}

		}
		}
		runTime->Stop();
}
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
	double kP;
	double kI;
	double kD;
	double kF;

	void TeleopInit() {

		updateRobotStatus();
		updateRobotPreference();

		runTime->Reset();

		crvbot.gyro->Reset();

		crvbot.leftEnc->Reset();
		crvbot.rightEnc->Reset();

		crvbot.fuelShooterMaster->SetPID(kP, kI, kD, kF);
		crvbot.fuelShooterSlave->SetPID(kP, kI, kD, kF);


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

			/*_____INTAKE CODE_____*/
			whilePressedAction(controllerButton(driverGamepad, Button::RightBumber), controllerButton(driverGamepad, Button::LeftBumber), crvbot.intakeRoller, 0.8);

			/*_____ALIGNMENT CODE_____*/
			//AlineCheck();

			/*_____UPDATING ROBOT STATUS_____*/
			updateRobotStatus();
		}

		runTime->Stop();

	}

/*	void DisabledInit()
	{
		stopRobot();
		resetEncouderCounts();
	}*/
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

	bool ReverseDirection = false;

	void DriveCode(void) {

	     double Left_Y  = controllerJoystick(driverGamepad, Axes::LEFT_Y);
	     double Right_Y = controllerJoystick(driverGamepad, Axes::RIGHT_Y);
		 double Right_X = (0.65*controllerJoystick(driverGamepad, Axes::RIGHT_X));

		 if(controllerButton(driverGamepad, Button::A))  ReverseDirection = true;
		 if(controllerButton(driverGamepad, Button::B))  ReverseDirection = false;

		 /*_________ Sets DriverJoystick in Tank Drive orientation _________*/
		 if(!ReverseDirection)
			 crvbot.robotDrive->SetLeftRightMotorOutputs((LEFT_MULTIPLER*(Left_Y - Right_X)), (RIGHT_MULTIPLER*(Left_Y + Right_X)));
		 /*_________ Sets DriverJoystick in FirstPerosnDrive orientation _________*/
	     else
	    	 crvbot.robotDrive->SetLeftRightMotorOutputs((LEFT_MULTIPLER*(-Left_Y - Right_X)), (RIGHT_MULTIPLER*(-Left_Y + Right_X)));
	}

	double hangerSpeed;

	void HangerCode(void) {
		whilePressedAction(controllerButton(operatorGamepad, Button::A), controllerButton(operatorGamepad, Button::B), crvbot.hangerMotor, hangerSpeed);

	}
	double	currentRPM;

	double setRPM;
	double agitatorspeed;
	double Error;
	double reverseTime = 0.5;
	double IntergralError = 0;

	bool AgitatorEnabled = false;
	bool ReachedRPM;

	void ShootingProcesses(void) {

		currentRPM = crvbot.fuelShooterMaster->GetEncVel();

		//Shooter is at negative RPM
		Error = setRPM  + currentRPM;
		IntergralError += Error;

		//			if((currentRPM > setRPM*.8) && (currentRPM < setRPM*1.2)){
		//				shooterMotorsSet( -(kI*IntergralError -kP*Error + setRPM));
		//			}
		//			else{

		//			}


		//AgitatorEnabled =  (crvbot.fuelShooterMaster->GetEncVel() <= -setRPM);

		if(AgitatorEnabled){

			crvbot.agitatorMotor->Set(agitatorspeed);
			ReachedRPM = true;
		}
		else
		{
			whilePressedAction(controllerButton(operatorGamepad, Button::RightBumber),
										   controllerButton(operatorGamepad, Button::LeftBumber),
										   crvbot.agitatorMotor, agitatorspeed);
			ReachedRPM = false;
		}

	}
	void updateRobotStatus(void) {

		SmartDashboard::PutNumber(" Total Runtime: ",        		  runTime->Get());
		SmartDashboard::PutNumber(" LeftMotor Current: ",   		  crvbot.leftFrontMotor->GetOutputCurrent());
		SmartDashboard::PutNumber(" RightMotor Current: ",  		  crvbot.rightFrontMotor->GetOutputCurrent());
		SmartDashboard::PutNumber(" LeftMotor Voltage: ",   		  crvbot.leftFrontMotor->GetOutputVoltage());
		SmartDashboard::PutNumber(" RightMotor Voltage: ",  		  crvbot.rightFrontMotor->GetOutputVoltage());
		SmartDashboard::PutNumber(" FuelShooter Voltage: ", 		  crvbot.fuelShooterMaster->GetOutputVoltage());
		SmartDashboard::PutNumber(" FuelShooter Current: ",  	 	  crvbot.fuelShooterMaster->GetOutputCurrent());
		SmartDashboard::PutNumber(" Agitator Voltage: ",              crvbot.agitatorMotor->GetOutputVoltage());
		SmartDashboard::PutNumber(" FuelShooter Voltage: ", 		  crvbot.fuelShooterSlave->GetOutputVoltage());
		SmartDashboard::PutNumber(" FuelShooter Current: ",  		  crvbot.fuelShooterSlave->GetOutputCurrent());
		SmartDashboard::PutNumber(" Agitator Current: ",              crvbot.agitatorMotor->GetOutputCurrent());
		SmartDashboard::PutNumber(" Agitator Current: ",              crvbot.agitatorMotor->GetOutputCurrent());
		SmartDashboard::PutNumber(" Left Side Encoder Count: ", 	  crvbot.leftEnc->GetRaw());
		SmartDashboard::PutNumber(" Right Side Encoder Count: ",      crvbot.rightEnc->GetRaw());
		SmartDashboard::PutNumber(" FuelShooter Encoder Position: ",  crvbot.fuelShooterMaster->GetEncPosition());
		SmartDashboard::PutNumber(" fuelShooterMaster Shooter Speed",      crvbot.fuelShooterMaster->Get());
		SmartDashboard::PutNumber(" fuelShooterMaster Mode",      		  crvbot.fuelShooterMaster->GetControlMode());
		SmartDashboard::PutNumber(" FuelShooter RPM: ",      		  currentRPM);
		SmartDashboard::PutNumber(" FuelShooter RPM Graph: ",      	  crvbot.fuelShooterMaster->GetEncVel());
		SmartDashboard::PutNumber(" FuelShooter Error:",              Error);
		SmartDashboard::PutNumber(" Gyro Angle : ", 				  crvbot.gyro->GetAngle());
		SmartDashboard::PutNumber(" Alignment : ",					  boilerPosition);
		SmartDashboard::PutBoolean(" ReverseDirection ", 			  ReverseDirection);
		SmartDashboard::PutBoolean(" Align to boiler ",               BoilerInRange);
		SmartDashboard::PutBoolean(" Reached Set RPM ",               ReachedRPM);
		SmartDashboard::PutNumber("IntergralError",                   IntergralError);
	}

	void updateRobotPreference(void) {

		leftMost 		  = prefs->GetInt("leftMost",     0);
		rightMost 		  = prefs->GetInt("rightMost",    500);
		setRPM		      = prefs->GetDouble("shooterspeed", .75);
		hangerSpeed       = prefs->GetDouble("Hanger Motor Speed", 0.75);
		agitatorspeed     = prefs->GetDouble("agitatorspeed", 0.25);
		kP	    		  = prefs->GetDouble("P", 1.0);
		kI	      		  = prefs->GetDouble("I", 0.00000001);
		kD	 			  = prefs->GetDouble("D", 0.0);
		kF				  = prefs->GetDouble("F", 0.04);
		auton 	  		  = prefs->GetInt("Auton", 1);

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
