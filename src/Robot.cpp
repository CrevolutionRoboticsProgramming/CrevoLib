#include "IterativeRobot.h"
#include "Joystick.h"
#include "RobotDrive.h"
#include "DriveTrain.h"
#include "CrevoRobot.h"
#include "OI.h"

class Robot: public IterativeRobot, public OI, public DriveTrain
{
private:

	CrevoRobot crvbot;
	Joystick *driverController,
			 *operatorController;

	void RobotInit()
	{
		crvbot.robotInit();
		driverController = new Joystick(0);
	}
	void AutonomousInit()
	{

	}

	void AutonomousPeriodic()
	{

	}

	void TeleopInit()
	{

	}

	void TeleopPeriodic()
	{
		while(IsOperatorControl() && IsEnabled())
		{
			double leftJoystick = controllerJoystick(driverController, Axes::LEFT_Y);
			double rightJoystick = controllerJoystick(driverController, Axes::RIGHT_Y);

			crvbot.robotDrive->SetLeftRightMotorOutputs( leftJoystick, rightJoystick);

			if(controllerButton(driverController, Button::A))
			{
				crvbot.armMotor->Set(Speed.FULL);
				SmartDashboard::PutNumber("ControllerJoystick", controllerJoystick(driverController, Axes::RIGHT_Y));
			}
			else if (controllerButton(driverController, Button::B))
			{
				crvbot.armMotor->Set(Speed.HALF);
			}
			else
			{
				crvbot.armMotor->Set(Speed.OFF);
				SmartDashboard::PutNumber("ControllerState", 0);
			}

		}
	}

	void TestPeriodic()
	{

	}
};

START_ROBOT_CLASS(Robot);
