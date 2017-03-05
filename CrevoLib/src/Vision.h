/*
 * Vision.h
 *
 *  Created on: Jan 17, 2017
 *      Author: Martin Smoger
 */

#ifndef SRC_VISION_H_
#define SRC_VISION_H_

#include <GripPipeline.h>

#include <WPILib.h>
#include <CrevoRobot.h>
#include <CameraServer.h>
#include <IterativeRobot.h>
#include <GripPipeline.h>
#include <networktables/NetworkTable.h>
#include <SmartDashboard/SmartDashboard.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>


class Vision {

private:
	GripPipeline gp;
	CrevoRobot crvbot;

	std::shared_ptr<NetworkTable> table;

	const int PIXEL_HEIGHT = 720;
	const int PIXEL_WIDTH  = 640;

	static void VisionTread(void);
	double calcDistancePixel(double reflectiveTapeArea);
public:
	double distanceFromBoiler(void);
	double alinementToBoiler(void);
	bool boilerDected(void);
	void startStream(void);
	void visionTrackingProcessing(void);
	Vision();
	virtual ~Vision();
};

#endif /* SRC_VISION_H_ */
