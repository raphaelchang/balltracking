#include <OpenNI.h>
#include "BallTracking.h"

int main(int argc, char** argv)
{
	openni::Status rc = openni::STATUS_OK;

	openni::Device device;
	openni::VideoStream depth;
	openni::VideoStream color;
	const char* deviceURI = openni::ANY_DEVICE;
	if (argc > 1)
	{
		deviceURI = argv[1];
	}

	rc = openni::OpenNI::initialize();

	rc = device.open(deviceURI);
	if (rc != openni::STATUS_OK)
	{
		printf("Device open failed: %s\n", openni::OpenNI::getExtendedError());
		openni::OpenNI::shutdown();
		return 1;
	}
//	device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
	device.setDepthColorSyncEnabled(true);

	rc = depth.create(device, openni::SENSOR_DEPTH);
	if (rc == openni::STATUS_OK)
	{
		rc = depth.start();
		if (rc != openni::STATUS_OK)
		{
			printf("Couldn't start depth stream: %s\n", openni::OpenNI::getExtendedError());
			depth.destroy();
		}
	}
	else
	{
		printf("Couldn't find depth stream: %s\n", openni::OpenNI::getExtendedError());
	}

	rc = color.create(device, openni::SENSOR_COLOR);
	if (rc == openni::STATUS_OK)
	{
		rc = color.start();
		if (rc != openni::STATUS_OK)
		{
			printf("Couldn't start color stream: %s\n", openni::OpenNI::getExtendedError());
			color.destroy();
		}
	}
	else
	{
		printf("Couldn't find color stream: %s\n", openni::OpenNI::getExtendedError());
	}

	if (!depth.isValid() || !color.isValid())
	{
		printf("No valid streams. Exiting\n");
		openni::OpenNI::shutdown();
		return 2;
	}

	BallTracking sampleViewer("Viewer", depth, color);

#ifdef GUI
	rc = sampleViewer.init(argc, argv);
	if (rc != openni::STATUS_OK)
	{
		openni::OpenNI::shutdown();
		return 3;
	}
	sampleViewer.run();
#else
	while (true)
	{
		sampleViewer.update();
	}
#endif
}
