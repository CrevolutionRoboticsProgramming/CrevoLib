/*
 * AutonVectors.cpp
 *
 *  Created on: Feb 15, 2017
 *      Author: Martin
 */

#include <AutonVectors.h>

AutonVectors::AutonVectors() {
	// TODO Auto-generated constructor stub

}

AutonVectors::~AutonVectors() {
	// TODO Auto-generated destructor stub
}

void AutonVectors::AutonSelect()
{

	SelectedAlliance = selectedAllianceSide.GetSelected();							//Get
	frc::SmartDashboard::PutString(SmartDashbaordAllianceKey, SelectedAlliance);

	autonSelected = chooser.GetSelected();

	if(autonSelected == GearCenter) 	       AutonSelected = SCORE_GEAR_CENTER;
	else if(autonSelected == GearLeft)		   AutonSelected = SCORE_GEAR_LEFT;
	else if(autonSelected == GearRight)        AutonSelected = SCORE_GEAR_RIGHT;
	else if(autonSelected == ShootFromWallRed) AutonSelected = SHOOT_FROM_WALL_CROSS_BASELINE;
	else if(autonSelected == GearCenterTimed)  AutonSelected = SCORE_GEAR_CENTER_TMED;
	else if(autonSelected == Baseline)	       AutonSelected = MOBILITY;
	else if(autonSelected == idle)			   AutonSelected = IDLE;
	else 									   AutonSelected = IDLE;

	frc::SmartDashboard::PutString(SmartDashboardAutonKey, autonSelected);

}

void AutonVectors::AutonStateProcess(void)
{

	switch(AutonSelected)
	{
	case SHOOT_FROM_WALL_CROSS_BASELINE:
	{
		DoShoot(ON);									//Being shooting process
		DoShoot(OFF);
		if(SelectedAlliance == redAlliance) { 			//Check to see what alliance we are at, Shooter is on the left of the robot

			crvbot.fuelShooterMaster->StopMotor();      //Stop shooter action
			crvbot.agitatorMotor->StopMotor();			//Stop agitator action
			drvt.driveByTime(.72, .5,-.5,Forward);		//Turn the robot out of the wall
			drvt.driveByTime(.5,0,0,Reverse);			//Make the Right side go more for certaintly
			drvt.driveByTime(3,.35,Reverse);			//Drive to the baseline
		}
		else if(SelectedAlliance == blueAlliance) {  	//Check to see what alliance we are at, Shooter is on the left of the robot

			crvbot.fuelShooterMaster->StopMotor();		//Stop shooter action
			crvbot.agitatorMotor->StopMotor();			//Stop agitator action
			drvt.driveByTime(.72, -.5,.5,Forward);		//Turn the robot out of the wall
			drvt.driveByTime(.5,0,0,Reverse);			//Make the Right side go more for certaintlY
			drvt.driveByTime(3,.35,Reverse);			//Drive to the baseline
			}
		AutonSelected = IDLE;							//Make the robot Idle so we can break out of the loop when we are running in auton
		break;
	}
	case SHOOT_FROM_HOPPER:
	{
		DoShoot(ON);									//Begin shooting process
		drvt.turnToHeading(-60, 0.5);					//
		drvt.moveRobot(1);

		break;
	}
	case MOBILITY:
	{
		drvt.driveByTime(3,.3,Forward);             //Drive for 3 seconds so we can garentee we have crossed the baseline
		AutonSelected = IDLE;                       //Make the robot Idle so we can break out of the loop when we are running in auton
		break;
	}
	case SCORE_GEAR_CENTER_SHOOT_FROM_BASELINE:
	case SCORE_GEAR_CENTER:
	{
		SmartDashboard::PutString("Auton Selected : ", "Score Gear Center");
		drvt.driveCountEncoder(A1_M1_LeftCount, A1_M1_Speed, Forward);
		AutonSelected = IDLE;
		break;
	}
	case SCORE_GEAR_CENTER_TMED:
	{
		crvbot.agitatorMotor->Set(-.3); 			//Lets start the agitaor to release the ceral box
		drvt.driveByTime(1.8,.35,.33, Forward);     //Now drive to the center peg
		drvt.driveByTime(.2,0.15,0, Forward);		//Move that left motor a bit so we can make sure we are straight on
		crvbot.agitatorMotor->StopMotor();			//Stop agitator motor so we can be safe that it has been released
		AutonSelected = IDLE;						//Make the robot Idle so we can break out of the loop when we are running in auton
		break;
	}
	case SCORE_GEAR_LEFT_SHOOT_FROM_BASELINE:
	case SCORE_GEAR_LEFT_KNOCK_DOWN_HOPPER:
	case SCORE_GEAR_LEFT:
	{
		drvt.driveCountEncoder(A2_M1_LeftCount, A2_M1_Speed, Forward);   //Start off the auton by driving to aline to the left peg
		drvt.turnToHeading(A2_M2_GyroAngle, A2_M2_Speed);				 //Lets turn so the ceral box is facing towards the left peg
		drvt.resetEncouderCounts();										 //Reset encoder counts so we can do our final action
		drvt.driveCountEncoder(A2_M3_LeftCount, A2_M3_Speed, Forward);	 //Drive to the left peg so Megan can get the gear
		AutonSelected = IDLE;											 //Make the robot Idle so we can break out of the loop when we are running in auton
		break;
	}
	case SCORE_GEAR_RIGHT_SHOOT_FROM_LIFT:
	case SCORE_GEAR_RIGHT:
	{
		drvt.driveCountEncoder(A3_M1_LeftCount, A3_M1_Speed, Forward);  //Start off the auton by driving to aline to the right peg
		drvt.turnToHeading(A3_M2_GyroAngle, A3_M2_Speed);				//Lets turn so the ceral box is facing towards the right peg
		drvt.resetEncouderCounts();										//Reset encoder counts so we can do our final action
		drvt.driveCountEncoder(A3_M3_LeftCount, A3_M3_Speed, Forward);	//Drive to the right peg so Megan can get the gear
		AutonSelected = IDLE;											//Make the robot Idle so we can break out of the loop when we are running in auton
		break;
	}
	case IDLE:
	{
		SmartDashboard::PutString(drvt.SmartDashbaordAction, "Idling");
		drvt.stopAndReset();											//Stop and reset encoders and drivetrain
		StopMotors();													//Stop all robot actions
		break;
	}
	}

}

void AutonVectors::IntakeState(MotorState state)
{
	if(state == ON)  crvbot.intakeRoller->Set(0.8);
	if(state == OFF) crvbot.intakeRoller->Set(0);
}

void AutonVectors::DoShoot(MotorState state)
{
	if(state == ON) {
		while(1 < 10) {
			crvbot.fuelShooterMaster->SetControlMode(CANSpeedController::kSpeed);
			crvbot.fuelShooterMaster->Set(-3350);
			Wait(2);
			crvbot.agitatorMotor->Set(-.3);
		}
	}

	else if(state == OFF) {
		crvbot.fuelShooterMaster->StopMotor();
		crvbot.agitatorMotor->StopMotor();
	}

}

void AutonVectors::DoMove(void)
{

}

void AutonVectors::DoAline(void)
{
	//vs.alinementToBoiler();
}

void AutonVectors::StopMotors(void)
{
	drvt.stopRobot();
	crvbot.agitatorMotor->StopMotor();
	crvbot.fuelShooterMaster->StopMotor();
	crvbot.hangerMotor->StopMotor();
	crvbot.intakeRoller->StopMotor();
}

