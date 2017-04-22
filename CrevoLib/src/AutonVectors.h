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

	std::string idle          	  		= "Idle";
	std::string GearCenter    	  		= "Score Gear Center";
	std::string	GearLeft          		= "Score Gear Left";
	std::string GearRight        	    = "Score Gear Right";
	std::string ShootFromWallRed 		= "Shoot from Wall Red";
	std::string ShootFromWallBlue 		= "Shoot from Wall Blue";
	std::string Baseline      	  		= "Baseline";
	std::string GearCenterTimed   		= "Score Gear Center Timed";

	std::string autonSelected;

	std::string blueAlliance 				= "Blue";
	std::string redAlliance 				= "Red";
	std::string SmartDashbaordAllianceKey   = "Alliance Side :";
	std::string SmartDashboardAutonKey      = "Auton Selected : ";


	std::string SelectedAlliance;

	SendableChooser<std::string> chooser;
	SendableChooser<std::string> selectedAllianceSide;

	/*/ 		Key for Auton Values:
	 * Ax = Defines what action the value belongs too
	 * Mx = Defines what move that the value belongs too
	/*/

	/*_____ Center Peg Gear Action Values _____*/
	const int    A1_M1_LeftCount  = 1785;
	const int    A1_M1_RightCount = 929;
	const double A1_M1_Speed      = 0.5;
	/*_________________________________________*/

	/*_____ Left Peg Gear Action Values _____*/
	const int     A2_M1_LeftCount = 1800;
	const int     A2_M1_RightCount = 830;
	const double  A2_M1_Speed     = 0.3;

	const int     A2_M2_GyroAngle = -56;
	const double  A2_M2_Speed     = 0.4;

	const int 	  A2_M3_LeftCount = 800;
	const int     A2_M3_RightCout = 1322;
	const double  A2_M3_Speed     = 0.3;
	/*_______________________________________*/

	/*_____ Right Peg Gear Action Values _____*/
	const int     A3_M1_LeftCount = 1700;
	const int     A3_M1_RightCount = 830;
	const double  A3_M1_Speed     = 0.3;

	const int     A3_M2_GyroAngle = 44;
	const double  A3_M2_Speed     = 0.3;

	const int 	  A3_M3_LeftCount = 650;
	const int     A3_M3_RightCout = 1322;
	const double  A3_M3_Speed     = 0.3;
	/*________________________________________*/

	/*_____ Hopper Dump Action Values _____*/
	/*_____________________________________*/

	/*_____ Shoot and cross baseline  Action Values _____*/
	const int SHOOTER_RPM = 3800;

	const int A4_M1_LeftCount = 800;
	const double A4_M1_Speed = 0.3;

	/*_____________________________________*/

	/*/
	* 1. Shoot from hopper
	* 2. Score gear right shoot from lift
	* 3. Score gear center, shoot from base line
	* 4. Score gear left, shooter from baseline
	* 5. Score gear left, knock down hoppers
	* 6. Hopper knock down
	/*/

	int AutonSelected;

	enum AutonStratagey { IDLE, MOBILITY, SHOOT_FROM_WALL_CROSS_BASELINE, SCORE_GEAR_CENTER, SCORE_GEAR_CENTER_TMED, SCORE_GEAR_LEFT, SCORE_GEAR_RIGHT, SHOOT_FROM_HOPPER, SCORE_GEAR_RIGHT_SHOOT_FROM_LIFT, SCORE_GEAR_CENTER_SHOOT_FROM_BASELINE, SCORE_GEAR_LEFT_SHOOT_FROM_BASELINE, SCORE_GEAR_LEFT_KNOCK_DOWN_HOPPER };

	enum AutonAction { SHOOT, FIND_BOILER, NOTHING};

	enum GamePiece { FUEL, GEAR };

	void AutonSelect(void);
	void AutonStateProcess(void);

	AutonVectors();
	virtual ~AutonVectors();

private:
	CrevoRobot crvbot;
	DriveTrain drvt;
	Vision     vs;

	void IntakeState(MotorState state);
	void DoMove(void);
	void StopMotors(void);
	void DoShoot(MotorState state);
	void DoAline(void);
};

#endif /* SRC_AUTONVECTORS_H_ */
