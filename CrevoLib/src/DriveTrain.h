#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include <iostream>
#include <stdio.h>
#include <unistd.h>

#include "crevoglb.h"
#include <CANTalon.h>
#include <Encoder.h>
#include <Timer.h>
#include <RobotDrive.h>
#include <AnalogGyro.h>
#include <AnalogAccelerometer.h>
#include <CrevoRobot.h>
#include <Math.h>

//{
 class DriveTrain
    {
 	 private:
	 	 	  CANTalon *rightFrontMotor;
			  CANTalon *rightRearMotor;
			  CANTalon *leftFrontMotor;
			  CANTalon *leftRearMotor;
			  CANTalon *freeTalon;
	  	  	  RobotDrive *robotDrive;
	  	  	  Timer *driveTime;
     	 	  AnalogGyro *gyro;
     	 	  AnalogAccelerometer *accel;
     	 	  Encoder *leftEnc;
			  Encoder *rightEnc;
     	 	  CrevoRobot crvbot;

	  	 	  const double DRV_TUNER = 1.15;
	  	 	  const double TRN_TUNER = 1.0;
	  	 	  const double TURN_TOLERANCE = 1.0;
	          const double VEH_WIDTH = crvbot.robotLentgh * TRN_TUNER;
	  	 	  const double WHL_DIAMETER = 6.6 * DRV_TUNER; //Diameter of the wheel (inches)
	          const int ENCODER_CPR = crvbot.encoderCPR;
	  	 	  const double GEAR_RATIO = 1;                   //Gear ratio
	          const double CIRCUMFERENCE = M_PI * WHL_DIAMETER;
	          const double CPI = ENCODER_CPR * GEAR_RATIO / CIRCUMFERENCE;
	  	 	  const double PADJ = 4.0;
	  	 	  const double PADJ_TURN = 0.0333;
	  	 	  const double THRESH = ((00.004 * 180) / M_PI);

	  	 	  int InverseMulti;
	  	 	  int autonCounter = 0;

	  	 	  bool FinishedPreviousTrial = false;
	  	 	  bool turnToheading;
	  	 	  bool leftCountsReached;
	  	 	  bool rightCountsReached;
	  	 	  bool IsMagEnc;

	          int distanceToCounts(double distance);
	          int angleToCounts(double angle, double radius);
	          bool gyroTurn(double angle, double pwr);
	          double getSteer(double error, double Pcoefficent);
	          double getGyroError(int targetHdg);
	          double errorClip(double number , double min, double max);

 	 public:
	          enum EncoderType {MagnetEncoder, QuadEncoder};
	          void init(RobotDrive *_robotDrvInit, AnalogGyro *_gyroInit, EncoderType enc);
	          void initMotors(CANTalon *_rightFront, CANTalon *_rightRear, CANTalon *_leftFront, CANTalon *_leftRear);
	          void initMotor(CANTalon *_selectedTalon);
	          void initEncoder(Encoder *_leftEncInit, Encoder *_rightEncInit);
	          void initDrive(RobotDrive *_robotDrvInit);
	          void initAccel(AnalogAccelerometer *_accelInit);
	          void moveRobot(double lPwr, double rPwr);
	          void moveRobot(double Pwr);
	          void moveMotor(CANTalon *_selectedTalon, double speed);
	          bool moveMotor(double pwr, double timeValue, Direction dir);
	          void stopRobot(void);
	          void driveDistanceEncoder(double dst, double pwr, Direction dir);
	          void driveCountEncoder(double enct, double pwr, Direction dir);
	          void driveByTime(double time, double pwr, Direction dir);
	          void encoderTurn(double angle, double pwr);
	          void turnToHeading(int heading, double pwr);
	          void stopAndReset(void);
	          void resetEncouderCounts(void);
	          DriveTrain();
	          virtual ~DriveTrain();
    };
//}
#endif
