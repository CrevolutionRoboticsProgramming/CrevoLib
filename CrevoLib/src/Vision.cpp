#include <Vision.h>
#include <SmartDashboard/SmartDashboard.h>
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
	/*_____ Select table object to revive the NetworkTable from GRIPn_____*/
	table->Initialize();
	table = NetworkTable::GetTable("GRIP/Crevo");


	/*_____ Starts Instance for Gear stream _____*/
	cs::UsbCamera camera2 = CameraServer::GetInstance()->StartAutomaticCapture(0);
	camera2.SetResolution(320, 280);

	/*_____ Starts Instance for Shooter stream on a Different Tread _____*/
	std::thread shooterStream(VisionTread);
	shooterStream.detach();
}

void Vision::VisionTread(void)
{
	cs::UsbCamera camera = CameraServer::GetInstance()->StartAutomaticCapture(1);
				// Set the resolution
	camera.SetResolution(320, 280);
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
	if(areas.size() > 0 && areas[0] > 100){
		Distance = (double)calcDistancePixel(areas[1]);
		return Distance;
	}
	else {
		return 0;
	}
	SmartDashboard::PutNumber("Distance From Boiler", Distance);
}

double Vision::alignmentToBoiler(void)
{
	double difference;

	/*_____ Pulls both the centerX and area _____*/
	std::vector<double> centerX  = table->GetNumberArray("centerX", llvm::ArrayRef<double>());
	std::vector<double> area  = table->GetNumberArray("area", llvm::ArrayRef<double>());

	/*_____ Checks to see if there is a countor detected and filers mixups _____*/
	if(centerX.size() > 0 && area[0] > 100)
		return centerX[0];
	else
		return 0;

	SmartDashboard::PutNumber("Robot Center X value:", difference);
}

double Vision::calcDistancePixel(double reflectiveTapeArea)
{
	double calculatedDistance;
	reflectiveTapeArea /= 40;
	return calculatedDistance = 25.098*pow(reflectiveTapeArea, -0.428);
}
