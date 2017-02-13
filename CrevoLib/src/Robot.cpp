#include <iostream>
#include <memory>
#include <string>
#include <stdio.h>
#include <unistd.h>

#include <IterativeRobot.h>
#include <Joystick.h>
#include <Timer.h>
#include <PIDController.h>
#include <Preferences.h>
#include <networktables/NetworkTable.h>
#include <CameraServer.h>

#include <OI.h>
#include <CrevoRobot.h>
#include <DriveTrain.h>
#include <Vision.h>
#include <FeedBack.h>

#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>

class Robot: public IterativeRobot, public OI, public DriveTrain
{
public:

	/*/
	 * 1. Shoot from hopper
	 * 2. Score gear right shoot from lift
	 * 3. Score gear center, shoot from base line
	 * 4. Score gear left, shooter from baseline
	 * 5. Score gear left, knock down hoppers
	 * 6. Hopper knock down
	 /*/

	//Put AutonNames here
	enum Autons{AutonMove, ForwardAndBackwards, EchyMemes, InternalScreams, VisionProcessingData};

	int AutonChooser;
	double speedShoot;
	bool tankTrue = false;
	bool visionDebug;

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
		/*/
		 *	Command to start up the stream from the USB camera.
		/*/
		vs.startStream();
		prefs = Preferences::GetInstance();

		//updateRobotStatus();

		std::cout << "_____________________________________________" << std::endl;
		std::cout << "" << std::endl;
		std::cout << "|| Crevobot || Robot completed initialize" << std::endl;
		std::cout << "_____________________________________________" << std::endl;
	}
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
	/*
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * GetString line to get the auto name from the text box below the Gyro.
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * if-else structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
	void AutonomousInit() override {
		/*/
		 *	Temporary: select what Auton you like to by its name. Will later be selected through the SmartDashboard.
		/*/
		AutonChooser = Autons::InternalScreams;
		/*/
		 * Initializes the robots settings into the DriveTrain class to use its functions.
		/*/
		initDrive(crvbot.robotDrive);

		updateRobotStatus();
		updateRobotPreference();

		initMotor(crvbot.intakeRoller);

		std::cout << "_____________________________________________" << std::endl;
		std::cout << "" << std::endl;
		std::cout << "|| Crevobot || In Autonomous Periodic Mode" << std::endl;
		std::cout << "_____________________________________________" << std::endl;

		runTime->Reset();
	}
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
	void AutonomousPeriodic() {
		runTime->Start();

		while(IsAutonomous() && IsEnabled())
		{
			updateRobotStatus();
			switch(AutonChooser)
			{
			case AutonMove:
			{
				driveByTime(2, Speed.Half, Forward);
				Wait(2);
				break;
			}
			case ForwardAndBackwards:
			{
				driveByTime(2, Speed.Half, Forward);
				Wait(2);
				driveByTime(2, Speed.Half , Reverse);
				Wait(2);
				break;
			}
			case VisionProcessingData:
			{
				while(true) { vs.distanceFromBoiler(); }
				break;
			}
			case EchyMemes:
			{
				break;
			}
			case InternalScreams:
			{
				while(IsEnabled())
				{
					moveRobot(0.5, -0.5);
				}
				break;
			}
			case 9:
			{
				break;
			}
			default:
			{
				crvbot.robotDrive->StopMotor();
				crvbot.fuelManipulator->StopMotor();
				crvbot.intakeRoller->StopMotor();
				crvbot.hangerMotor->StopMotor();
			}
			}
		}
		runTime->Stop();
	}
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
	void TeleopInit() {

		updateRobotStatus();
		updateRobotPreference();

		std::cout << "_____________________________________________" << std::endl;
		std::cout << "" << std::endl;
		std::cout << "|| Crevobot || In TeleopPeriodic Mode" << std::endl;
		std::cout << "_____________________________________________" << std::endl;

		runTime->Reset();
	}
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
	bool shooterPressed;
	bool stillPressed;
	bool toggled = false;
	bool lastToggled = false;

	void TeleopPeriodic() {
		runTime->Start();
		while(IsOperatorControl() && IsEnabled())
		{
		//	DriveCode();
		//	toggleAction((driverGamepad->GetRawAxis(2) > 0.1), crvbot.fuelManipulator, speedShoot);

			if(controllerButton(driverGamepad, Button::A)){
				driverGamepad->SetRumble(Joystick::RumbleType::kRightRumble, 1);
			}
			else{
				driverGamepad->SetRumble(Joystick::RumbleType::kRightRumble, 0);
			}
			updateRobotStatus();
			Wait(0.005);
		}
		runTime->Stop();
	}
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

	void TestPeriodic() {
		lw->Run();
	}
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
	void DriveCode()
		{
			double Left_Y  = controllerJoystick(driverGamepad, 	   	 Axes::LEFT_Y);
			double Right_Y = controllerJoystick(driverGamepad, 		 Axes::RIGHT_Y);
			double Right_X = (0.65*controllerJoystick(driverGamepad, Axes::RIGHT_X));

			if(controllerButton(driverGamepad, Button::A))  tankTrue = true;
			if(controllerButton(driverGamepad, Button::B))  tankTrue = false;

			/*_________ Sets DriverJoystick in Tank Drive orientation _________*/
			if(tankTrue) crvbot.robotDrive->SetLeftRightMotorOutputs(LEFT_MULTIPLER*Left_Y, RIGHT_MULTIPLER*Right_Y);
			/*_________ Sets DriverJoystick in FirstPerosnDrive orientation _________*/
			else         crvbot.robotDrive->SetLeftRightMotorOutputs((LEFT_MULTIPLER*(Left_Y - Right_X)), (RIGHT_MULTIPLER*(Left_Y + Right_X)));

			//driverGamepad->SetRumble(Joystick::RumbleType::kRightRumble, 0.5);


		}

	void SpeedScale()
	{
		if(controllerButton(driverGamepad, Button::A) && !stillPressed)
		{
			speedShoot = speedShoot - 0.05;
			stillPressed = true;
			Wait(0.05);
		}
		else if(controllerButton(driverGamepad, Button::B) && !stillPressed)
		{
			speedShoot = speedShoot + 0.05;
			stillPressed = true;
			Wait(0.05);
		}
		else
		{
			stillPressed = false;
		}
	}

void updateRobotStatus(void)
{
	SmartDashboard::PutNumber(" Total Runtime: ",        		  runTime->Get());
	SmartDashboard::PutNumber(" SpeedShooter: ", 		 		  speedShoot);
    SmartDashboard::PutBoolean(" TankDrive: ", 			 	  	  tankTrue);
	SmartDashboard::PutNumber(" LeftMotor Current: ",   		  crvbot.leftFrontMotor->GetOutputCurrent());
	SmartDashboard::PutNumber(" RightMotor Current: ",  		  crvbot.rightFrontMotor->GetOutputCurrent());
	SmartDashboard::PutNumber(" LeftMotor Voltage: ",   		  crvbot.leftFrontMotor->GetOutputVoltage());
	SmartDashboard::PutNumber(" RightMotor Voltage: ",  		  crvbot.rightFrontMotor->GetOutputVoltage());
	SmartDashboard::PutNumber(" FuelShooter Voltage: ", 		  crvbot.fuelManipulator->GetOutputVoltage());
	SmartDashboard::PutNumber(" FuelShooter Current: ",  	 	  crvbot.fuelManipulator->GetOutputCurrent());
	SmartDashboard::PutNumber(" Left Side Encoder Count: ", 	  crvbot.leftEnc->GetRaw());
	SmartDashboard::PutNumber(" Right Side Encoder Count: ",      crvbot.rightEnc->GetRaw());
	SmartDashboard::PutNumber(" FuelShooter Encoder Position: ",  crvbot.fuelManipulator->GetEncPosition());
	SmartDashboard::PutNumber(" FuleShooter RPM: ",      		  crvbot.fuelManipulator->GetEncVel());
}

void updateRobotPreference(void)
{
	speedShoot   = prefs->GetDouble("Shooter Speed Scale", 0.4);
	tankTrue     = prefs->GetBoolean("Is tankDrive on?", true);
	AutonChooser = prefs->GetInt("Choose Auton", 9);
	crvbot.kP	 = prefs->GetDouble("P", 0.0);
	crvbot.kI	 = prefs->GetDouble("I", 0.0);
	crvbot.kD	 = prefs->GetDouble("D", 0.0);
}

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

};

START_ROBOT_CLASS(Robot)
