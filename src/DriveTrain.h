#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include "CANTalon.h"
#include "Encoder.h"
#include "RobotDrive.h"
#include "AnalogGyro.h"
#include <Math.h>

//namespace DriveTrain
//{
 class DriveTrain
    {
 	 private:
	 	 	  const double kCPI = 0;
	  	 	  const double kDRV_TUNER = 1.15;
	  	 	  const double kTRN_TUNER = 1.0;
	  	 	  const double kTURN_TOLERANCE = 1.0;
	          //const double VEH_WIDTH;// = ShelbyBot.BOT_WIDTH * TRN_TUNER;
	  	 	  const double kWHL_DIAMETER = 6.6 * kDRV_TUNER; //Diameter of the wheel (inches)
	          //const int ENCODER_CPR = ShelbyBot.ENCODER_CPR;
	  	 	  const double kGEAR_RATIO = 1;                   //Gear ratio
	          const double kCIRCUMFERENCE = M_PI * kWHL_DIAMETER;
	          //Cons double CPI = ENCODER_CPR * GEAR_RATIO / CIRCUMFERENCE;
	  	 	  const double kPADJ = 4.0;
	  	 	  const double kPADJ_TURN = 0.0333;
	  	 	  const double kTHRESH = ((00.004 * 180) / M_PI);

	  	 	  bool leftCountsReached;
	  	 	  bool rightCountsReached;

	  	  	  CANTalon *rightFrontMotor,
	                   *rightRearMotor,
	                   *leftFrontMotor,
	                   *leftRearMotor;

	          RobotDrive *robotDrive;

	          AnalogGyro *gyro;

	          Encoder *leftEnc,
	                  *rightEnc;

	          int distanceToCounts(double distance);
	          int angleToCounts(double angle, double radius);
	          void resetEncouderCounts(void);

 	 public:
	          enum Direction {Forward, Reverse};
	          void initMotors(CANTalon *rightFront, CANTalon *rightRear, CANTalon *leftFront, CANTalon *leftRear);
	          void init(RobotDrive *robotDrvInit, AnalogGyro *gyroInit, Encoder *leftEncInit, Encoder *rightEncInit);
	          void moveRobot(double lPwr, double rPwr);
	          void moveRobot(double Pwr);
	          void stopRobot(void);
	          void driveDistanceEncoder(double dst, double pwr, Direction dir);
	          void encoderTurn(double angle, double pwr);
	          void turnToHeading(int heading);
	          void stopAndReset(void);
	          DriveTrain();
	          virtual ~DriveTrain();
    };
//}
#endif
