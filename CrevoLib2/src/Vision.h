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

public:
	void initStream(bool streamOn);
	static void VisionTread();
	Vision();
	virtual ~Vision();
};

#endif /* SRC_VISION_H_ */
