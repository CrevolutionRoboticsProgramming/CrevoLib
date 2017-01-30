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
	//Put AutonNames here
	enum Autons{AutonMove, ForwardAndBackwards, EchyMemes, VisionProcessing};

	int AutonChooser;
	double speedShoot;
	bool tankTrue = false;
	bool visionDebug;

/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
	void RobotInit() override {

		driverGamepad = new Joystick(0);
		operatorGamepad = new Joystick(1);
		//speedShoot = prefs->GetDouble("Shooter Speed Scale", 0.4);
		crvbot.robotInit();
		/*
		 * Calibrates Gyro, needs two seconds in order to calibrate correctly.
		 */
		//crvbot.gyro->Calibrate();
		//Wait(2);

		//nTable = NetworkTable::GetTable("Grip/VSReporting");
/*
		if(fork() == 0)
		{
			system("/home/lvuser/grip &");
		}

*/
		/*
		 *	Command to start up the stream from the usb camera. Can be disabled through SmartDashboard by setting the streamOn boolean to false.
		 */
		//vs.startStream();
		//prefs = Preferences::GetInstance();

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
		/*
		 *	Temporary: select what Auton you like to by its name. Will later be selected through the SmartDashboard.
		 */
		AutonChooser = Autons::ForwardAndBackwards;
		//visionDebug = prefs->GetBoolean("Debug", false);
		/*
		 * Initializes the robots settings into the DriveTrain class to use its functions.
		 */
		initDrive(crvbot.robotDrive);


		crvbot.gyro->Reset();
		crvbot.robotDrive->StopMotor();

		 auto grip = NetworkTable::GetTable("grip");

		        /* Get published values from GRIP using NetworkTables */
		 auto areas = grip->GetNumberArray("targets/area", llvm::ArrayRef<double>());


		 for (auto area : areas) {
			 std::cout << "Got contour with area=" << area << std::endl;
		  }

	}
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
	void AutonomousPeriodic() {

		SmartDashboard::PutBoolean(" Debug: ", visionDebug);

		while(IsAutonomous() && IsEnabled())
		{
			switch(AutonMove)
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
			case VisionProcessing:
			{

				auto grip = NetworkTable::GetTable("grip");
				auto XValue = grip->GetNumberArray("centerX", llvm::ArrayRef<double>());
				auto YValue = grip->GetNumberArray("centerY", llvm::ArrayRef<double>());

			}

			}
		}

	}
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
	void TeleopInit() {
		//speedShoot = prefs->GetDouble("Shooter Speed Scale", 0.4);
		//tankTrue = prefs->GetBoolean("Is tankDrive on?", true);
		//visionDebug = prefs->GetBoolean("Debug", false);

	}
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
	bool shooterPressed;
	bool stillPressed;
	bool toggled = false;
	bool lastToggled = false;
	void TeleopPeriodic() {
		while(IsOperatorControl() && IsEnabled())
		{

			DriveCode();
			//SpeedScale();
			//toggleAction((driverGamepad->GetRawAxis(2) > 0.1), crvbot.fuelManipulator, speedShoot);

			SmartDashboard::PutNumber("SpeedShooter find me", speedShoot);
			SmartDashboard::PutBoolean(" TankDrive ", tankTrue);
			Wait(0.005);
		}
	}
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

	void TestPeriodic() {
		lw->Run();
	}
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
	void DriveCode()
		{
			double Left_Y = controllerJoystick(driverGamepad, Axes::LEFT_Y);
			double Right_Y = controllerJoystick(driverGamepad, Axes::RIGHT_Y);
			double Right_X =  controllerJoystick(driverGamepad, Axes::RIGHT_X);

			if(controllerButton(driverGamepad, Button::A))
				tankTrue = true;
			else if(controllerButton(driverGamepad, Button::B))
				tankTrue = false;

			if(tankTrue)
				crvbot.robotDrive->SetLeftRightMotorOutputs(Left_Y, Right_Y);
			else
				crvbot.robotDrive->SetLeftRightMotorOutputs(Left_Y - Right_X, Left_Y + Right_X);

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

/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

private:
	LiveWindow* lw = LiveWindow::GetInstance();

	CrevoRobot crvbot;
	Joystick *driverGamepad;
	Joystick *operatorGamepad;
	Preferences *prefs;
	std::shared_ptr<NetworkTable> nTable;

	Vision vs;
};

START_ROBOT_CLASS(Robot)
