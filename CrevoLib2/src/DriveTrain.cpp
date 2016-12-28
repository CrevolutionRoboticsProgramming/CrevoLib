#include <DriveTrain.h>
#include <Math.h>

 DriveTrain::DriveTrain()
 {

 }

 DriveTrain::~DriveTrain()
 {

 }

 int DriveTrain::distanceToCounts(double distance)
 {
	 return (int)(distance * kCPI);
 }

    //use for testing

 void DriveTrain::initMotors(CANTalon *rightFront, CANTalon *rightRear, CANTalon *leftFront, CANTalon *leftRear)
 {
	 rightFrontMotor = rightFront;
	 rightRearMotor = rightRear;
	 leftFrontMotor = leftFront;
	 leftRearMotor = leftRear;
 }

 void DriveTrain::init(RobotDrive *robotDrvInit, AnalogGyro *gyroInit, Encoder *leftEncInit, Encoder *rightEncInit)
 {
	 robotDrive = robotDrvInit;
     gyro = gyroInit;
     leftEnc = leftEncInit;
     rightEnc = rightEncInit;
 }

 void DriveTrain::moveRobot(double lPwr, double rPwr)
 {
	 robotDrive->SetLeftRightMotorOutputs(lPwr, rPwr);
 }
 void DriveTrain::moveRobot(double Pwr)
  {
 	 robotDrive->SetLeftRightMotorOutputs(Pwr, Pwr);
  }


 void DriveTrain::stopRobot(void)
 {
	 robotDrive->SetLeftRightMotorOutputs(0, 0);
 }


 void DriveTrain::driveDistanceEncoder(double dst, double pwr, Direction dir)
 {
	 int counts = distanceToCounts(dst);

	 if (dir == DriveTrain::Reverse)
     {
		 counts *= -1;
     }
	 int lft_target = leftEnc->GetRaw() + counts;
     int rgt_target = rightEnc->GetRaw() + counts;

     while(lft_target  != leftEnc->GetRaw() && rgt_target != rightEnc->GetRaw())
     {
    	 moveRobot(pwr);
     }
 }

 void DriveTrain::encoderTurn(double angle, double pwr)
 {
	 int counts = angleToCounts(angle, (1./2.0));
	 int lft_target = leftEnc->GetRaw() - counts;
	 int rgt_target = rightEnc->GetRaw() + counts;

	 while(lft_target  != leftEnc->GetRaw() && rgt_target != rightEnc->GetRaw())
	 {
		 moveRobot(pwr);
	 }
 }

 int DriveTrain::angleToCounts(double angle, double radius)
 {
	 return distanceToCounts((angle*(M_PI/180)) * radius);
 }

 void DriveTrain::resetEncouderCounts()
 {
	 leftEnc->Reset();
	 rightEnc->Reset();
 }

 void DriveTrain::stopAndReset()
 {
	 stopRobot();
	 resetEncouderCounts();
 }


