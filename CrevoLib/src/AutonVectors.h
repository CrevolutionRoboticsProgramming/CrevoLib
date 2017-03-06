/*
 * AutonVectors.h
 *
 *  Created on: Feb 15, 2017
 *      Author: Martin
 */

#ifndef SRC_AUTONVECTORS_H_
#define SRC_AUTONVECTORS_H_

#include <crevoglb.h>
#include <iostream>
#include <CrevoRobot.h>
#include <DriveTrain.h>
#include <Math.h>

class AutonVectors {
public:

	/*/ 		Key for Auton Values:
	 * Ax = Defines what action the value belongs too
	 * Mx = Defines what move that the value belongs too
	/*/

	/*_____ Center Peg Gear Action Values _____*/
	const int A1_M1_LeftCount = 2728;
	const int A1_M1_RightCount = 929;
	const int A1_M1_Speed     = 0.5;
	/*_________________________________________*/

	/*_____ Left Peg Gear Action Values _____*/
	const int A2_M1_LeftCount = 2254;
	const int A2_M1_RightCount = 830;
	const int A2_M1_Speed     = 0.5;

	const int A2_M2_GyroAngle = -58;
	const int A2_M2_Speed     = 0.4;

	const int A2_M3_LeftCount = 5215;
	const int A2_M3_RightCout = 1322;
	const int A2_M3_Speed     = 0.5;
	/*_______________________________________*/

	/*_____ Right Peg Gear Action Values _____*/
	/*________________________________________*/

	/*_____ Hopper Dump Action Values _____*/
	/*_____________________________________*/


	/*/
	* 1. Shoot from hopper
	* 2. Score gear right shoot from lift
	* 3. Score gear center, shoot from base line
	* 4. Score gear left, shooter from baseline
	* 5. Score gear left, knock down hoppers
	* 6. Hopper knock down
	/*/

	enum AutonStratagey { IDLE, SCORE_GEAR_CENTER, SCORE_GEAR_LEFT, SCORE_GEAR_RIGHT, SHOOT_FROM_HOPPER, SCORE_GEAR_RIGHT_SHOOT_FROM_LIFT, SCORE_GEAR_CENTER_SHOOT_FROM_BASELINE, SCORE_GEAR_LEFT_SHOOT_FROM_BASELINE, SCORE_GEAR_LEFT_KNOCK_DOWN_HOPPER };

	enum AutonAction { SHOOT, FIND_BOILER, NOTHING};

	enum GamePiece { FUEL, GEAR };

	void AutonSelect(AutonStratagey strat);
	bool AutonStateProcess(void);

	AutonVectors();
	virtual ~AutonVectors();

private:
	CrevoRobot crvbot;
	DriveTrain drvt;
	Vision     vs;

	void IntakeState(MotorState state);
	void DoMove(void);
	void StopMotors(void);
	void DoShoot(void);
	void DoAline(void);
};

#endif /* SRC_AUTONVECTORS_H_ */
