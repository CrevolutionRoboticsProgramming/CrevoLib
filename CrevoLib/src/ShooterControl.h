/*
 * ShooterControl.h
 *
 *  Created on: Mar 2, 2017
 *      Author: Martin
 */

#ifndef SRC_SHOOTERCONTROL_H_
#define SRC_SHOOTERCONTROL_H_

#include "CrevoRobot.h"
#include "PIDController.h"

class ShooterControl {
public:

	CrevoRobot crvbot;

	ShooterControl();
	virtual ~ShooterControl();
};

#endif /* SRC_SHOOTERCONTROL_H_ */
