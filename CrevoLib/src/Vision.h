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
#include <CameraServer.h>
#include <IterativeRobot.h>
#include <GripPipeline.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>


class Vision {

private:
	GripPipeline gp;
	static void VisionTread();
public:
	void startStream(void);
	void visionTrackingProcessing(void);
	Vision();
	virtual ~Vision();
};

#endif /* SRC_VISION_H_ */
