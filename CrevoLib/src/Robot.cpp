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
	int AutonChooser;

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
		init(crvbot.robotDrive, crvbot.gyro, EncoderType::QuadEncoder);


		updateRobotStatus();
		updateRobotPreference();

		runTime->Reset();

		std::cout << "_____________________________________________" << std::endl;
		std::cout << "" << std::endl;
		std::cout << "|| Crevobot || In Autonomous Periodic Mode" << std::endl;
		std::cout << "_____________________________________________" << std::endl;
		hasReached = false;
	}
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
	bool hasReached;
	int BoilerPostition;

	void AutonomousPeriodic() {

		runTime->Start();

		AutonSelect(AutonStratagey::SHOOT_FROM_HOPPER);

		while(IsAutonomous() && IsEnabled())
		{
			updateRobotStatus();


//				while(vs.alinementToBoiler() <=  200  && !hasReached)
//				{
//					drvt.moveRobot(0.15, -0.15);
//				}				hasReached = true;
//				drvt.moveRobot(0,0);

		}

		runTime->Stop();
	}
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
	int kP;
	int kI;
	int kD;
	int kF;

	void TeleopInit() {

		updateRobotStatus();
		updateRobotPreference();

		runTime->Reset();

		crvbot.gyro->Reset();

		crvbot.leftEnc->Reset();
		crvbot.rightEnc->Reset();


		//crvbot.fuelShooter1->SetP(kP);
		//crvbot.fuelShooter1->SetI(kI);
		//crvbot.fuelShooter1->SetD(kD);
		//crvbot.fuelShooter1->SetF(kF);

		//crvbot.fuelShooter2->SetP(kP);
		//crvbot.fuelShooter2->SetI(kI);
		//crvbot.fuelShooter2->SetD(kD);
		//crvbot.fuelShooter2->SetF(kF);

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
	Command *Alliance_COLOR;


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

	int	currentRPM;

	double setRPM;
	double agitatorspeed;
	double Error;
	double reverseTime = 0.5;

	bool AgitatorEnabled = false;
	bool ReachedRPM;

	void ShootingProcesses(void) {

		if(operatorGamepad->GetRawAxis(2) > 0)  shooterMotorsSet(-setRPM);
		if(operatorGamepad->GetRawAxis(3) > 0)  shooterMotorsSet(setRPM);
		else									shooterMotorsSet(0);


		AgitatorEnabled =  (crvbot.fuelShooter1->GetEncVel() <= -20000);

		if(AgitatorEnabled){

			crvbot.agitatorMotor->Set(agitatorspeed);
			ReachedRPM = true;
		}
		else
		{
			whilePressedAction(controllerButton(operatorGamepad, Button::LeftBumber),
										   controllerButton(operatorGamepad, Button::RightBumber),
										   crvbot.agitatorMotor, agitatorspeed);
			ReachedRPM = false;
		}

	}

	void shooterMotorsSet(double inValue){

		crvbot.fuelShooter1->Set(inValue);
		crvbot.fuelShooter2->Set(inValue);
	}

	void updateRobotStatus(void) {

		SmartDashboard::PutNumber(" Total Runtime: ",        		  runTime->Get());
		SmartDashboard::PutNumber(" LeftMotor Current: ",   		  crvbot.leftFrontMotor->GetOutputCurrent());
		SmartDashboard::PutNumber(" RightMotor Current: ",  		  crvbot.rightFrontMotor->GetOutputCurrent());
		SmartDashboard::PutNumber(" LeftMotor Voltage: ",   		  crvbot.leftFrontMotor->GetOutputVoltage());
		SmartDashboard::PutNumber(" RightMotor Voltage: ",  		  crvbot.rightFrontMotor->GetOutputVoltage());
		SmartDashboard::PutNumber(" FuelShooter Voltage: ", 		  crvbot.fuelShooter1->GetOutputVoltage());
		SmartDashboard::PutNumber(" FuelShooter Current: ",  	 	  crvbot.fuelShooter1->GetOutputCurrent());
		SmartDashboard::PutNumber(" Agitator Voltage: ",              crvbot.agitatorMotor->GetOutputVoltage());
		SmartDashboard::PutNumber(" FuelShooter Voltage: ", 		  crvbot.fuelShooter2->GetOutputVoltage());
		SmartDashboard::PutNumber(" FuelShooter Current: ",  		  crvbot.fuelShooter2->GetOutputCurrent());
		SmartDashboard::PutNumber(" Agitator Current: ",              crvbot.agitatorMotor->GetOutputCurrent());
		SmartDashboard::PutNumber(" Agitator Current: ",              crvbot.agitatorMotor->GetOutputCurrent());
		SmartDashboard::PutNumber(" Left Side Encoder Count: ", 	  crvbot.leftEnc->GetRaw());
		SmartDashboard::PutNumber(" Right Side Encoder Count: ",      crvbot.rightEnc->GetRaw());
		SmartDashboard::PutNumber(" FuelShooter Encoder Position: ",  crvbot.fuelShooter1->GetEncPosition());
		SmartDashboard::PutNumber(" FuelShooter RPM: ",      		  crvbot.fuelShooter1->GetEncVel());
		SmartDashboard::PutNumber(" FuelShooter RPM: ",      		  crvbot.fuelShooter1->GetEncVel());
		SmartDashboard::PutNumber(" FuelShooter Error:",              crvbot.fuelShooter1->GetClosedLoopError());
		SmartDashboard::PutNumber(" Gyro Angle : ", 				  crvbot.gyro->GetAngle());
		SmartDashboard::PutNumber(" Alignment : ",					  boilerPosition);
		SmartDashboard::PutBoolean(" ReverseDirection : ", 			  ReverseDirection);
		SmartDashboard::PutBoolean(" Align to boiler ",               BoilerInRange);
		SmartDashboard::PutBoolean(" Reached Set RPM ",               ReachedRPM);
	}

	void updateRobotPreference(void) {

		AutonChooser      = prefs->GetInt("Choose Auton", 9);
		leftMost 		  = prefs->GetInt("leftMost",     0);
		rightMost 		  = prefs->GetInt("rightMost",    500);
		setRPM		      = prefs->GetDouble("shooterspeed", .75);
		agitatorspeed     = prefs->GetDouble("agitatorspeed", 0.25);
		kP	    		  = prefs->GetDouble("P", 1.0);
		kI	      		  = prefs->GetDouble("I", 0.0);
		kD	 			  = prefs->GetDouble("D", 0.0);
		kF				  = prefs->GetDouble("F", 0.04);

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
			if(boilerPosition <= 0)
				BoilerInRange = false;
			else
				BoilerInRange = true;
		}
	}
};

START_ROBOT_CLASS(Robot)
