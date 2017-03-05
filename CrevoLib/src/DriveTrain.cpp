#include <DriveTrain.h>

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
     std::cout << "Robot Initialized" << std::endl;
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
 void DriveTrain::initMotor(CANTalon *_selectedTalon)
 {
	 freeTalon = _selectedTalon;
	 driveTime = new Timer();
 }
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

 void DriveTrain::initDrive(RobotDrive *_robotDrvInit)
  {
	 robotDrive = _robotDrvInit;
	 driveTime 		= new Timer();
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
 bool DriveTrain::moveMotor(double pwr, double timeValue, Direction dir)
   {
	 	driveTime->Start();
	 	 if(dir == Direction::Forward)
	 	 {
	 		 pwr *= -1;
	 	 }

	 	 while(driveTime->Get() < timeValue)
	 	 {
	 		 freeTalon->Set(pwr);
	 	 }
	 	freeTalon->Set(0);
	 	 driveTime->Stop();
	 	 return false;
   }
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
 void DriveTrain::stopRobot(void)
  {
	 moveRobot(0);
  }
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

 void DriveTrain::driveDistanceEncoder(double dst, double pwr, Direction dir)
  {
	 double counts = distanceToCounts(dst);
	 if (dir == Reverse)
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
	 stopAndReset();
	 std::cout << "____________________________________________________________________________________________________" << std::endl;
	 std::cout << "" << std::endl;
	 std::cout << "|| Crevobot || Action Completed With : Rgt Dst: " << rightEnc->GetDistance() << "Lft Dst: "<< rightEnc->GetDistance() <<" Speed: "  << abs(pwr) << "Direction: "<< dir << std::endl;
	 std::cout << "" << std::endl;
  }

 void DriveTrain::driveCountEncoder(double enct, double pwr, Direction dir)
 {

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
	 		int lft_target = leftEnc->GetRaw() + enct;
	 		int rgt_target = rightEnc->GetRaw() + enct;
	 		if (dir == Reverse)
	 		{
	 			lft_target *= -1;
	 			rgt_target *= -1;
	 			pwr *= -1;
	 			while(lft_target  <= leftEnc->GetRaw() && rgt_target <= rightEnc->GetRaw())
	 			{
	 				moveRobot(pwr);
	 			}
	 		}

	 		else
	 		{
	 			 while(lft_target  >= leftEnc->GetRaw() && rgt_target >= rightEnc->GetRaw())
	 			 {
	 				 moveRobot(pwr);
	 			 }
	 		}
	 	 }
	 	stopAndReset();
	 	std::cout << "____________________________________________________________________________________________________" << std::endl;
	 	std::cout << "" << std::endl;
	 	std::cout << "|| Crevobot || Action Completed With : Rgt Enc: " << rightEnc->GetRaw() << "Lft Enc: "<< leftEnc->GetRaw() <<" Speed: "  << abs(pwr) << "Direction: "<< dir << std::endl;
	 	std::cout << "" << std::endl;
 }

/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
 void DriveTrain::driveByTime(double timeValue, double pwr, Direction dir)
 {
	 driveTime->Start();
	 if(dir == Direction::Reverse)
	 {
		 pwr *= -1;
	 }
	 while(driveTime->Get() < timeValue)
	 {
		 moveRobot(pwr);
	 }
	 stopRobot();
	 driveTime->Stop();
	 std::cout << "____________________________________________________________________________________________________" << std::endl;
	 std::cout << "" << std::endl;
	 std::cout << "|| Crevobot || Action Completed With : Time Driven: " << driveTime->Get() << " Speed: "  << abs(pwr) << "Direction: "<< dir << std::endl;
	 std::cout << "" << std::endl;
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
	 std::cout << "____________________________________________________________________________________________________" << std::endl;
	 std::cout << "" << std::endl;
	 std::cout << "|| Crevobot || Action Completed With: angle " << counts << "Speed: " << pwr << std::endl;
	 std::cout << "" << std::endl;
 }
 /*-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

 void DriveTrain::turnToHeading(int heading, double pwr)
 {
	 heading = round(heading - 0.0);
	 while(!gyroTurn(heading, pwr));
	 stopAndReset();
 }

/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
 bool DriveTrain::gyroTurn(double angle, double pwr)
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

		 steer = getSteer(error, PADJ_TURN);
		 rgtPwr = pwr * steer;
		 errorClip(rgtPwr, -1, 1);
		 lftPwr = -rgtPwr;
	 }

	 moveRobot(lftPwr, rgtPwr);
	 return onTarget;
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

 double DriveTrain::getSteer(double error, double Pcoefficent)
 {
	 return errorClip(error * Pcoefficent , -1, 1);
 }

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
double DriveTrain::errorClip(double number, double min, double max)
{
	if(number < min) return min;
	if(number > max) return max;
	return number;
}

