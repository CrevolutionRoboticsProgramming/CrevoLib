/*
 * Vision.cpp
 *
 *  Created on: Jan 17, 2017
 *      Author: Martin Smoger
 */

#include <Vision.h>

#include <WPILib.h>
#include <CameraServer.h>
#include <IterativeRobot.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>

Vision::Vision() {
	// TODO Auto-generated constructor stub

}

Vision::~Vision() {
	// TODO Auto-generated destructor stub
}

void Vision::startStream(void)
{
	std::thread visionThread(VisionTread);
	visionThread.detach();
}

void Vision::VisionTread()
{
	cs::UsbCamera camera = CameraServer::GetInstance()->StartAutomaticCapture();
				// Set the resolution
	camera.SetResolution(640, 480);

				// Get a CvSink. This will capture Mats from the Camera
	cs::CvSink cvSink = CameraServer::GetInstance()->GetVideo();
				// Setup a CvSource. This will send images back to the Dashboard
	cs::CvSource outputStream = CameraServer::GetInstance()->PutVideo("Stream", 720, 640);

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
					// Put a rectangle on the image
			rectangle(mat, cv::Point(100, 100), cv::Point(400, 400),
							cv::Scalar(255, 255, 255), 5);
					// Give the output stream a new image to display
			outputStream.PutFrame(mat);
		}
}
