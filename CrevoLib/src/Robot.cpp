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

		driverGamepad   = new Joystick(0);
		operatorGamepad = new Joystick(1);
		runTime         = new Timer();

		crvbot.robotInit();
		/*/
		 * Calibrates Gyro, needs two seconds in order to calibrate correctly.
		/*/
		/*/
		 *	Command to start up the stream from the USB camera.
		/*/
		vs.startStream();
		prefs = Preferences::GetInstance();

		std::cout << "____________________________________________________________________________________________________" << std::endl;
		std::cout << "|| Crevobot || Robot completed initialize" << std::endl;
		std::cout << "" << std::endl;
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
		AutonChooser = Autons::VisionProcessingData;
		/*/
		 * Initializes the robots settings into the DriveTrain class to use its functions.
		/*/
		//initDrive(crvbot.robotDrive);

		speedShoot  = prefs->GetDouble("Shooter Speed Scale", 0.4);
		tankTrue    = prefs->GetBoolean("Is tankDrive on?", true);
		visionDebug = prefs->GetBoolean("Debug", false);

		std::cout << "____________________________________________________________________________________________________" << std::endl;
		std::cout << "|| Crevobot || In Autonomous Periodic Mode" << std::endl;
		std::cout << "" << std::endl;

		runTime->Reset();
	}
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
	void AutonomousPeriodic() {
		runTime->Start();
		updateRobotStatus();
		while(IsAutonomous() && IsEnabled())
		{
			switch(AutonChooser)
			{
			case AutonMove:
			{
				driveByTime(0.3, Speed.Half, Forward);
				Wait(2);
				break;
			}
			case ForwardAndBackwards:
			{
				driveByTime(0.3, Speed.Half, Forward);
				Wait(2);
				driveByTime(0.3, Speed.Half , Reverse);
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
				while(1) { crvbot.robotDrive->SetLeftRightMotorOutputs(0.5, -0.5); }
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

		//updateRobotStatus();
		speedShoot  = prefs->GetDouble("Shooter Speed Scale", 0.4);
		tankTrue    = prefs->GetBoolean("Is tankDrive on?", true);
		visionDebug = prefs->GetBoolean("Debug", false);

		std::cout << "____________________________________________________________________________________________________" << std::endl;
		std::cout << "|| Crevobot || In TeleopPeriodic Mode" << std::endl;
		std::cout << "" << std::endl;

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
			DriveCode();

			//SpeedScale();
			//toggleAction((driverGamepad->GetRawAxis(2) > 0.1), crvbot.fuelManipulator, speedShoot);
			//updateRobotStatus();
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
			double Right_X = shape(controllerJoystick(driverGamepad, Axes::RIGHT_X));

			if(controllerButton(driverGamepad, Button::A))  tankTrue = true;
			if(controllerButton(driverGamepad, Button::B))  tankTrue = false;

			/*_________ Sets DriverJoystick in Tank Drive orientation _________*/
			if(tankTrue) crvbot.robotDrive->SetLeftRightMotorOutputs(Left_Y, Right_Y);
			/*_________ Sets DriverJoystick in FirstPerosnDrive orientation _________*/
			else         crvbot.robotDrive->SetLeftRightMotorOutputs(Left_Y - Right_X, Left_Y + Right_X);

			//driverGamepad->SetRumble(Joystick::RumbleType::kRightRumble, 0.5);


		}

	void SpeedScale()
	{
		if(controllerButton(driverGamepad, Button::A) && !stillPressed)
		{
			speedShoot = speedShoot - 0.05;
			stillPressed = true;
			Wait(0.5);
		}
		else if(controllerButton(driverGamepad, Button::B) && !stillPressed)
		{
			speedShoot = speedShoot + 0.05;
			stillPressed = true;
			Wait(0.5);
		}
		else
		{
			stillPressed = false;
		}
	}

void updateRobotStatus(void)
{
	SmartDashboard::PutNumber(" Total Runtime: ", runTime->Get());
	SmartDashboard::PutNumber(" LeftMotor Current: ", crvbot.leftFrontMotor->GetOutputCurrent());
	SmartDashboard::PutNumber(" RightMotor Current: ", crvbot.rightFrontMotor->GetOutputCurrent());
	SmartDashboard::PutNumber(" LeftMotor Voltage: ", crvbot.leftFrontMotor->GetOutputVoltage());
	SmartDashboard::PutNumber(" RightMotor Voltage: ", crvbot.rightFrontMotor->GetOutputVoltage());
	SmartDashboard::PutNumber(" SpeedShooter: ", speedShoot);
	SmartDashboard::PutBoolean(" Debug: ", visionDebug);
	SmartDashboard::PutBoolean(" TankDrive: ", tankTrue);
}

/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

private:
	LiveWindow* lw = LiveWindow::GetInstance();

	CrevoRobot crvbot;
	Joystick *driverGamepad;
	Joystick *operatorGamepad;
	Preferences *prefs;
	Timer *runTime;
	Vision vs;
};

START_ROBOT_CLASS(Robot)
