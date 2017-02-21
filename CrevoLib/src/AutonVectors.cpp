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

void AutonVectors::AutonSwitch(AutonStratagey strat)
{
	switch(strat)
	{
	case SHOOT_FROM_HOPPER:
	case SCORE_GEAR_RIGHT_SHOOT_FROM_LIFT:
	case SCORE_GEAR_CENTER_SHOOT_FROM_BASELINE:
	case SCORE_GEAR_LEFT_SHOOT_FROM_BASELINE:
	case SCORE_GEAR_LEFT_KNOCK_DOWN_HOPPER:
	case IDLE:
			break;
	}
	switch(strat)
	{
	case SHOOT_FROM_HOPPER:
	case SCORE_GEAR_RIGHT_SHOOT_FROM_LIFT:
	case SCORE_GEAR_CENTER_SHOOT_FROM_BASELINE:
	case SCORE_GEAR_LEFT_SHOOT_FROM_BASELINE:
	case SCORE_GEAR_LEFT_KNOCK_DOWN_HOPPER:
	case IDLE:
			break;
	}
	switch(strat)
	{
	case SHOOT_FROM_HOPPER:
	case SCORE_GEAR_RIGHT_SHOOT_FROM_LIFT:
	case SCORE_GEAR_CENTER_SHOOT_FROM_BASELINE:
	case SCORE_GEAR_LEFT_SHOOT_FROM_BASELINE:
	case SCORE_GEAR_LEFT_KNOCK_DOWN_HOPPER:
	case IDLE:
			break;
	}
	switch(strat)
	{
	case SHOOT_FROM_HOPPER:
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
	MakeMove();
	switch(int action)
	{
	case SHOOT:
		DoShoot();
		break;
	case SCORE_GEAR:
		break;
	case DUMP_HOPPER:
		break;
	case FIND_BOILER:
		break;
	case IDLE:
	}
}

void AutonVectors::IntakeState(MotorState state)
{
	if(state == ON)  crvbot.intakeRoller(0.8);
	if(state == OFF) crvbot.intakeRoller(0);
}

void AutonVectors::DoShoot(void)
{

}

void AutonVectors::MakeMove(void)
{

}
