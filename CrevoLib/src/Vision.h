/*
 * Vision.h
 *
 *  Created on: Jan 17, 2017
 *      Author: Martin Smoger
 */

#ifndef SRC_VISION_H_
#define SRC_VISION_H_

class Vision {
private:
	static void VisionTread();
public:
	void startStream(void);

	Vision();
	virtual ~Vision();
};

#endif /* SRC_VISION_H_ */
