#include <iostream>
#include <memory>
#include <string>

#include <IterativeRobot.h>
#include <Joystick.h>
#include <Timer.h>
#include <PIDController.h>
#include <Preferences.h>

#include <OI.h>
#include <CrevoRobot.h>
#include <DriveTrain.h>

#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>

class Robot: public IterativeRobot, public OI, public DriveTrain
{
public:
	//Put AutonNames here
	enum Autons{AutonMove, ForwardAndBackwards};

	SendableChooser<std::string> chooser;

	int AutonChooser;
	double speedShoot = prefs->GetDouble("Shooter Speed Scale", 0.4);
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
	void RobotInit() {
		driverGamepad = new Joystick(0);
		operatorGamepad = new Joystick(1);
		crvbot.robotInit();
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

		AutonChooser = Autons::ForwardAndBackwards;
		initDrive(crvbot.robotDrive);

		//pidController.SetOutputRange(0, 5);
		//pidController.SetSetpoint(kSetPoint[0]);

		crvbot.robotDrive->StopMotor();

	}
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
	void AutonomousPeriodic() {
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

			}
		}

	}
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
	void TeleopInit() {

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
			SpeedScale();

			toggleAction((driverGamepad->GetRawAxis(2) > 0.1), crvbot.fuelManipulator, speedShoot);

			SmartDashboard::PutNumber("SpeedShooter find me", speedShoot);
			SmartDashboard::PutBoolean("DriveTrain State", tankTrue);

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
				crvbot.robotDrive->SetLeftRightMotorOutputs(Left_Y + Right_X, Left_Y - Right_X);


			SmartDashboard::PutNumber("Driver Joystick Left Y Axis ", Left_Y);
			SmartDashboard::PutNumber("Driver Joystick Right Y Axis ", Right_Y);
			SmartDashboard::PutNumber("Driver Joystick Right X Axis ", Right_X);
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


	//constexpr std::array<double, 3> kSetPoint = {1.0, 2.6, 4.3};


	bool tankTrue = false;
};

START_ROBOT_CLASS(Robot)
