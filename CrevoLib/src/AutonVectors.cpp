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

void AutonVectors::AutonSelect(AutonStratagey strat)
{
	switch(strat)
	{
	case SHOOT_FROM_HOPPER:
	case SCORE_GEAR_CENTER:
	{
		//drvt.driveCountEncoder(A1_M1_LeftCount, A1_M1_Speed, Forward);
		break;
	}
	case SCORE_GEAR_LEFT:
	{
		//drvt.driveCountEncoder(A2_M1_LeftCount, A2_M1_Speed, Forward);
		//drvt.turnToHeading(A2_M2_GyroAngle, A2_M2_Speed);
		//drvt.driveCountEncoder(A2_M3_LeftCount, A2_M3_Speed, Forward);
		break;
	}
	case SCORE_GEAR_RIGHT:
	{
		break;
	}
	case SCORE_GEAR_RIGHT_SHOOT_FROM_LIFT:
	case SCORE_GEAR_CENTER_SHOOT_FROM_BASELINE:
	case SCORE_GEAR_LEFT_SHOOT_FROM_BASELINE:
	case SCORE_GEAR_LEFT_KNOCK_DOWN_HOPPER:
	case IDLE:
		break;
	}
}

bool AutonVectors::AutonStateProcess(void)
{
	DoMove();
	switch(1)
	{
	case SHOOT:
		DoShoot();
		break;
	case FIND_BOILER:
		DoAline();
		break;
	case NOTHING:
		StopMotors();
		break;
	}
	return true;
}

void AutonVectors::IntakeState(MotorState state)
{
	if(state == ON)  crvbot.intakeRoller->Set(0.8);
	if(state == OFF) crvbot.intakeRoller->Set(0);
}

void AutonVectors::DoShoot(void)
{

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
	//drvt.stopRobot();
	crvbot.agitatorMotor->StopMotor();
	crvbot.fuelShooterMaster->StopMotor();
	crvbot.hangerMotor->StopMotor();
	crvbot.intakeRoller->StopMotor();
}

