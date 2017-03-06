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

#include <crevoglb.h>
#include <DriveTrain.h>
#include <AutonVectors.h>

#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>

class Robot: public IterativeRobot, public OI, public AutonVectors
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

		vs.startStream();

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

	enum Autons{AutonMove, ForwardAndBackwards, EchyMemes, InternalScreams, VisionProcessingData};

	void AutonomousInit() override {

		/*/
		 *	Temporary: select what Auton you like to by its name. Will later be selected through the SmartDashboard.
		/*/
		AutonChooser = Autons::AutonMove;
//		/*/
//		 * Initializes the robots settings into the DriveTrain class to use its functions.
//		/*/
		crvbot.leftEnc->Reset();
		crvbot.rightEnc->Reset();

		crvbot.gyro->Reset();

		/*/
		 *  Init Shooter as controlled by velocity
		/*/

		crvbot.fuelManipulator->SetTalonControlMode(CANTalon::TalonControlMode::kSpeedMode);

		//drvt.initDrive(crvbot.robotDrive);
		drvt.init(crvbot.robotDrive, crvbot.gyro, drvt.EncoderType::QuadEncoder);


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


				while(vs.alinementToBoiler() <=  200  && !hasReached)
				{
					drvt.moveRobot(0.15, -0.15);
				}				hasReached = true;
				drvt.moveRobot(0,0);

		}

		runTime->Stop();
	}
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
	void TeleopInit() {

		updateRobotStatus();
		updateRobotPreference();

		runTime->Reset();

		crvbot.gyro->Reset();

		crvbot.leftEnc->Reset();
		crvbot.rightEnc->Reset();

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
			ShootProcesses();

			/*_____INTAKE CODE_____*/
			whilePressedAction(controllerButton(driverGamepad, Button::RightBumber), controllerButton(driverGamepad, Button::LeftBumber), crvbot.intakeRoller, 0.8);

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


	bool reverseDirection = false;

	void DriveCode(void) {

	     double Left_Y  = controllerJoystick(driverGamepad, 	   	 Axes::LEFT_Y);
	     double Right_Y = controllerJoystick(driverGamepad, 		 Axes::RIGHT_Y);
		 double Right_X = (0.65*controllerJoystick(driverGamepad, Axes::RIGHT_X));

		 if(controllerButton(driverGamepad, Button::A))  reverseDirection = true;
		 if(controllerButton(driverGamepad, Button::B))  reverseDirection = false;

		 /*_________ Sets DriverJoystick in Tank Drive orientation _________*/
		 if(!reverseDirection)
			 crvbot.robotDrive->SetLeftRightMotorOutputs((LEFT_MULTIPLER*(Left_Y - Right_X)), (RIGHT_MULTIPLER*(Left_Y + Right_X)));
		 /*_________ Sets DriverJoystick in FirstPerosnDrive orientation _________*/
	     else
	    	 crvbot.robotDrive->SetLeftRightMotorOutputs((LEFT_MULTIPLER*(-Left_Y - Right_X)), (RIGHT_MULTIPLER*(-Left_Y + Right_X)));
	}


	bool shooterEnabled;
	void ShootProcesses(void) {

		toggleAction((operatorGamepad->GetRawAxis(3) > 0), crvbot.fuelManipulator, 1);

		if(crvbot.fuelManipulator->GetEncVel() >= 12000)
			whilePressedAction(controllerButton(operatorGamepad, Button::LeftBumber), controllerButton(operatorGamepad, Button::RightBumber), crvbot.agitatorMotor, 0.4);
		else
			crvbot.fuelManipulator->Set(0);
	}

	void updateRobotStatus(void) {

		SmartDashboard::PutNumber(" Total Runtime: ",        		  runTime->Get());
		SmartDashboard::PutNumber(" LeftMotor Current: ",   		  crvbot.leftFrontMotor->GetOutputCurrent());
		SmartDashboard::PutNumber(" RightMotor Current: ",  		  crvbot.rightFrontMotor->GetOutputCurrent());
		SmartDashboard::PutNumber(" LeftMotor Voltage: ",   		  crvbot.leftFrontMotor->GetOutputVoltage());
		SmartDashboard::PutNumber(" RightMotor Voltage: ",  		  crvbot.rightFrontMotor->GetOutputVoltage());
		SmartDashboard::PutNumber(" FuelShooter Voltage: ", 		  crvbot.fuelManipulator->GetOutputVoltage());
		SmartDashboard::PutNumber(" FuelShooter Current: ",  	 	  crvbot.fuelManipulator->GetOutputCurrent());
		SmartDashboard::PutNumber(" Agitator Voltage: ",              crvbot.agitatorMotor->GetOutputVoltage());
		SmartDashboard::PutNumber(" Agitator Current: ",              crvbot.agitatorMotor->GetOutputCurrent());
		SmartDashboard::PutNumber(" Agitator Current: ",              crvbot.agitatorMotor->GetOutputCurrent());
		SmartDashboard::PutNumber(" Left Side Encoder Count: ", 	  crvbot.leftEnc->GetRaw());
		SmartDashboard::PutNumber(" Right Side Encoder Count: ",      crvbot.rightEnc->GetRaw());
		SmartDashboard::PutNumber(" Left Side Rate :", 				  crvbot.leftEnc->GetRate());
		SmartDashboard::PutNumber(" RightSide Rate :" ,               crvbot.rightEnc->GetRate());
		SmartDashboard::PutNumber(" FuelShooter Encoder Position: ",  crvbot.fuelManipulator->GetEncPosition());
		SmartDashboard::PutNumber(" FuleShooter RPM: ",      		  crvbot.fuelManipulator->GetEncVel());
		//SmartDashboard::PutNumber(" RobotCenter X Value: ",           vs.alinementToBoiler());
		SmartDashboard::PutNumber(" Gyro Angle : ", 				  crvbot.gyro->GetAngle());

		SmartDashboard::PutBoolean(" ReverseDirection : ", 			  reverseDirection);
	}

	void updateRobotPreference(void) {

		reverseDirection  = prefs->GetBoolean("Is tankDrive on?", true);
		BoilerPostition   = prefs->GetInt("BoilerPostition ", 300);
		AutonChooser      = prefs->GetInt("Choose Auton", 9);
		crvbot.kP	      = prefs->GetDouble("P", 0.0);
		crvbot.kI	      = prefs->GetDouble("I", 0.0);
		crvbot.kD	 	  = prefs->GetDouble("D", 0.0);
	}

};

START_ROBOT_CLASS(Robot)
