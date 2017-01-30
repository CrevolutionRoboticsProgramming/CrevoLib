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

	std::thread shooterStream(VisionTread);
	shooterStream.detach();
}

void Vision::VisionTread(void)
{
	cs::UsbCamera camera = CameraServer::GetInstance()->StartAutomaticCapture(0);
				// Set the resolution
	camera.SetResolution(400, 250);
				// Get a CvSink. This will capture Mats from the Camera
/*
	cs::CvSink cvSink = CameraServer::GetInstance()->GetVideo();
				// Setup a CvSource. This will send images back to the Dashboard
	cs::CvSource outputStream = CameraServer::GetInstance()->PutVideo("Shooter Stream Camera", 600, 400);

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
*/

	cs::UsbCamera camera2 = CameraServer::GetInstance()->StartAutomaticCapture(1);
	camera2.SetResolution(400, 250);

}

void Vision::visionTrackingProcessing(void)
{

}
