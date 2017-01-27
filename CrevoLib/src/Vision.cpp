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

void Vision::startStreamShooter(void)
{

	std::thread shooterStream(VisionTreadShooterCamera);
	shooterStream.detach();
}
void Vision::startStreamGear(void)
{
	std::thread gearStream(VisionTreadGearCamera);
	gearStream.detach();

}

void Vision::VisionTreadShooterCamera(void)
{
	cs::UsbCamera camera = CameraServer::GetInstance()->StartAutomaticCapture(0);
				// Set the resolution
	camera.SetResolution(640, 480);
				// Get a CvSink. This will capture Mats from the Camera
	cs::CvSink cvSink = CameraServer::GetInstance()->GetVideo();
				// Setup a CvSource. This will send images back to the Dashboard
	cs::CvSource outputStream = CameraServer::GetInstance()->PutVideo("Shooter Stream Camera", 720, 640);

				// Mats are very memory expensive. Lets reuse this Mat.
	cv::Mat mat;

	while (true) {
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

void Vision::VisionTreadGearCamera(void)
{
	cs::UsbCamera camera = CameraServer::GetInstance()->StartAutomaticCapture(1);
				// Set the resolution
	camera.SetResolution(640, 480);

				// Get a CvSink. This will capture Mats from the Camera
	cs::CvSink cvSink = CameraServer::GetInstance()->GetVideo();
				// Setup a CvSource. This will send images back to the Dashboard
	cs::CvSource outputStream = CameraServer::GetInstance()->PutVideo("Front Stream Camera", 720, 640);

				// Mats are very memory expensive. Lets reuse this Mat.
	cv::Mat mat;

	while (true) {
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

void Vision::visionTrackingProcessing(void)
{

}
