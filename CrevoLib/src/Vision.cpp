/*
 * Vision.cpp
 *
 *  Created on: Jan 17, 2017
 *      Author: Martin Smoger
 */

#include <Vision.h>

Vision::Vision() {
	// TODO Auto-generated constructor stub

}

Vision::~Vision() {
	// TODO Auto-generated destructor stub
}

void Vision::startStream(void)
{
	/*_____ Select table object to revive the NetworkTable from GRIPn_____*/
	table = NetworkTable::GetTable("GRIP/Crevo");


	/*_____ Starts Instance for Gear stream _____*/
	cs::UsbCamera camera2 = CameraServer::GetInstance()->StartAutomaticCapture(1);
	camera2.SetResolution(640, 480);

	/*_____ Starts Instance for Shooter stream on a Different Tread _____*/
	std::thread shooterStream(VisionTread);
	shooterStream.detach();
}

void Vision::VisionTread(void)
{
	cs::UsbCamera camera = CameraServer::GetInstance()->StartAutomaticCapture(0);
				// Set the resolution
	camera.SetResolution(640, 480);
				// Get a CvSink. This will capture Mats from the Camera

	cs::CvSink cvSink = CameraServer::GetInstance()->GetVideo();
				// Setup a CvSource. This will send images back to the Dashboard
	cs::CvSource outputStream = CameraServer::GetInstance()->PutVideo("Crevo Cam", 320, 280);

				// Mats are very memory expensive. Lets reuse this Mat.
	cv::Mat mat;

	while (true){

					// Tell the CvSink to grab a frame from the camera and put it
					// in the source mat.  If there is an error notify the output.
			if (cvSink.GrabFrame(mat) == 0) {
						// Send the output the error.
				outputStream.NotifyError(cvSink.GetError());
						// skip the rest of the current iteration
				continue;
			}
					// Give the output stream a new image to display
			outputStream.PutFrame(mat);

		}

}

double Vision::distanceFromBoiler(void)
{
	double Distance;
	std::cout << "Areas: ";

	/*____ Pulls area array from the NetworkTable ____*/
	std::vector<double> areas = table->GetNumberArray("area", llvm::ArrayRef<double>());

	/*_____ Run through for each value in array _____*/
	for(unsigned int i = 0; i < areas.size(); i++) {
			std::cout << " | " << areas[i] <<" | ";
			Distance = calcDistancePixel(areas[1]);   /*_____ Calculate distance from boiler using area_____*/
			std::cout << " | Distance: " << Distance;
	}
	std::cout << std::endl;
	SmartDashboard::PutNumber("Distance From Boiler", Distance);
	return Distance;
}

double Vision::alinementToBoiler(void)
{
	double difference;

	/*_____ Pulls both the centerX and area _____*/
	std::vector<double> centerX  = table->GetNumberArray("centerX", llvm::ArrayRef<double>());
	std::vector<double> area  = table->GetNumberArray("area", llvm::ArrayRef<double>());

	/*_____ Checks to see if there is a countor detected and filers mixups _____*/
	if(centerX.size() > 0 && area[0] > 100)
	{
		std::cout << "Difference from Center: ";
		for(unsigned int i = 0; i < centerX.size(); i++)
		{
			std::cout << " | " << centerX[i] << " | ";
		}
		std::cout << std::endl;
		difference = centerX[0];
		return difference;
	}
	else{
		std::cout << "Nothing Detected : " << std::endl;
		return 0;
	}

	SmartDashboard::PutNumber("Robot Center X value:", difference);
}

double Vision::calcDistancePixel(double reflectiveTapeArea)
{
	double calculatedDistance;
	reflectiveTapeArea /= 40;
	return calculatedDistance = 25.098*pow(reflectiveTapeArea, -0.428);
}
