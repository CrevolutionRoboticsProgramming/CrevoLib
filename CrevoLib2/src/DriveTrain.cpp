#include <DriveTrain.h>
#include <Math.h>

 DriveTrain::DriveTrain()
 {

 }

 DriveTrain::~DriveTrain()
 {

 }
    //use for testing
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
 void DriveTrain::init(RobotDrive *_robotDrvInit, AnalogGyro *_gyroInit, EncoderType enc)
  {
	 robotDrive = _robotDrvInit;
     gyro = _gyroInit;

     if(enc == MagnetEncoder)
     {
    	 IsMagEnc = true;
     }
     else
     {
    	 IsMagEnc = false;
     }

  }
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
 void DriveTrain::initEncoder(Encoder *_leftEncInit, Encoder *_rightEncInit)
 {
	 leftEnc = _leftEncInit;
	 rightEnc = _rightEncInit;
 }
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
 void DriveTrain::initMotors(CANTalon *_rightFront, CANTalon *_rightRear, CANTalon *_leftFront, CANTalon *_leftRear)
  {
	 rightFrontMotor = _rightFront;
	 rightRearMotor = _rightRear;
	 leftFrontMotor = _leftFront;
	 leftRearMotor = _leftRear;
  }
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

 void DriveTrain::initDrive(RobotDrive *_robotDrvInit)
  {
	 robotDrive = _robotDrvInit;
  }
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

 void DriveTrain::initAccel(AnalogAccelerometer *_accelInit)
  {
	 accel = _accelInit;
  }
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

 void DriveTrain::moveRobot(double lPwr, double rPwr)
  {
	 robotDrive->SetLeftRightMotorOutputs(lPwr, rPwr);
  }
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
 void DriveTrain::moveRobot(double Pwr)
  {
	 robotDrive->SetLeftRightMotorOutputs(Pwr, Pwr);
  }

 void DriveTrain::moveMotor(CANTalon *_selectedTalon, double speed)
  {
	 freeTalon = _selectedTalon;
	 freeTalon->Set(speed);
  }
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
 void DriveTrain::stopRobot(void)
  {
	 moveRobot(0);
  }
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

 void DriveTrain::driveDistanceEncoder(double dst, double pwr, Direction dir)
  {
	 int counts = distanceToCounts(dst);

	 if (dir == DriveTrain::Reverse)
     {
		 counts *= -1;
		 pwr *= -1;
     }

	 if(IsMagEnc)
	 {
		// int lft_target = leftFrontMotor->GetEncPosition(CANTalon::) + counts;
//		 int rgt_target = rightFrontMotor->GetEncPosition() + counts;
//
//		 while(lft_target  != leftFrontMotor->GetEncPosition() && rgt_target != rightFrontMotor->GetEncPosition())
//		 {
//			 moveRobot(pwr);
//		 }
	 }
	 else
	 {
		 int lft_target = leftEnc->GetRaw() + counts;
		 int rgt_target = rightEnc->GetRaw() + counts;

		 while(lft_target  != leftEnc->GetRaw() && rgt_target != rightEnc->GetRaw())
		 {
			 moveRobot(pwr);
		 }
	 }
	 stopRobot();
 }
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
 void DriveTrain::driveByTime(double timeValue, double pwr, Direction dir)
 {
	 timeValue *= 500;

	 if(dir == Direction::Reverse)
	 {
		 pwr *= -1;
	 }

	 if(FinishedPreviousTrial)
	 {
		 autonCounter = 0;
		 FinishedPreviousTrial = false;
	 }

	 while(autonCounter < timeValue)
	 {
		 moveRobot(pwr);
		 autonCounter++;
	 }
	 FinishedPreviousTrial = true;
	 moveRobot(0);
 }
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
 void DriveTrain::encoderTurn(double angle, double pwr)
 {
	 // Turn on Axle, left is positive
	 int counts = angleToCounts(angle, (VEH_WIDTH/2.0));
	 int lftTarget = leftEnc->GetRaw() - counts;
	 int rgtTarget = rightEnc->GetRaw() + counts;

	 while(lftTarget  != leftEnc->GetRaw() && rgtTarget != rightEnc->GetRaw())
	 {
		 moveRobot(pwr);
	 }
	 stopRobot();
 }
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
 void DriveTrain::gyroTurn(double angle, double pwr)
 {
	 double error;
	 double steer;
	 bool onTarget = false;
	 double lftPwr;
	 double rgtPwr;

	 error = getGyroError((int)angle);

	 if(abs(error) <= TURN_TOLERANCE)
	 {
		 steer = 0.0;
		 lftPwr = 0.0;
		 rgtPwr = 0.0;
		 onTarget = true;
	 }
	 else
	 {
		// steer = g
	 }
 }
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

 int DriveTrain::angleToCounts(double angle, double radius)
 {
	 return distanceToCounts((angle*(M_PI/180)) * radius);
 }
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

 void DriveTrain::resetEncouderCounts()
 {
	 leftEnc->Reset();
	 rightEnc->Reset();
 }
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

 int DriveTrain::distanceToCounts(double distance)
 {
	 return (int)(distance * CPI);
 }
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

 void DriveTrain::stopAndReset()
 {
	 stopRobot();
	 resetEncouderCounts();
 }
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

 double DriveTrain::getGyroError(int targetHdg)
 {
	 int error = targetHdg - gyro->GetAngle();
	 while(error > 180) error -= 360;
	 while(error <= -180) error+= 360;
	 return error;
 }
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

