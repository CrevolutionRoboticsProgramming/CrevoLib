/*
 * AutonVectors.h
 *
 *  Created on: Feb 15, 2017
 *      Author: Martin
 */

#ifndef SRC_AUTONVECTORS_H_
#define SRC_AUTONVECTORS_H_

#include <crevoglb.h>
#include <CrevoRobot.h>
#include <DriveTrain.h>
#include <Math.h>

class AutonVectors {
public:

	enum AutonStratagey { IDLE, SHOOT_FROM_HOPPER, SCORE_GEAR_RIGHT_SHOOT_FROM_LIFT, SCORE_GEAR_CENTER_SHOOT_FROM_BASELINE, SCORE_GEAR_LEFT_SHOOT_FROM_BASELINE, SCORE_GEAR_LEFT_KNOCK_DOWN_HOPPER };

	enum AutonAction { SHOOT, SCORE_GEAR, DUMP_HOPPER, NOTHING, FIND_BOILER};

	enum GamePiece { FUEL, GEAR };

	void AutonChooser(AutonStratagey strat);
	bool AutonStateProcess(void);

	AutonVectors();
	virtual ~AutonVectors();

private:
	CrevoRobot crvbot;
	DriveTrain drvt;
	Vision     vs;

	void IntakeState(MotorState state);
	void DoMove(void);
	void DoGear(void);
	void DoShoot(void);
	void DoAline(void);
};

#endif /* SRC_AUTONVECTORS_H_ */
