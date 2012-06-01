#include "ActivityMap.h"


int main()
{
	ActivityMap actMapCreator(2);

	KinectSensor kinects[2];
	const XnDepthPixel* depthMaps[2];
	const XnRGB24Pixel* rgbMaps[2];

	kinects[0].initDevice(1, REF_CAM, true);
	kinects[1].initDevice(2, REF_CAM, true);

	kinects[0].startDevice();
	kinects[1].startDevice();

	namedWindow("Activity Map");
	bool bShouldStop = false;
	bool trans = true;

//	VideoWriter w ("out.avi", CV_FOURCC('M','J','P','G'), 25, actMapCreator.getResolution(), true);

	/*Mat rgbImg   (XN_VGA_Y_RES, XN_VGA_X_RES, CV_8UC3);
	Mat depthImg (XN_VGA_Y_RES, XN_VGA_X_RES, CV_8UC3);*/

	int cont = 0;
	while (!bShouldStop && cont++ < 50)
	{
		kinects[0].waitAndUpdate();
		kinects[1].waitAndUpdate();

		depthMaps[0] = kinects[0].getDepthMap();
		depthMaps[1] = kinects[1].getDepthMap();
		rgbMaps[0] = kinects[0].getRGBMap();
		rgbMaps[1] = kinects[1].getRGBMap();

	/*	Utils::convertXnDepthPixelToFrame(depthMaps[1], depthImg);
		Utils::convertXnRGB24PixelToFrame(rgbMaps[1], rgbImg);

		imwrite("outDepth_cam2.jpg", depthImg);
		imwrite("outRGB_cam2.jpg", rgbImg);*/

		Mat* activityMap = actMapCreator.createActivityMap(kinects, depthMaps, rgbMaps, trans);
		
		imshow("Activity Map", *activityMap);
		
//		w << *activityMap;
//		imwrite("out.jpg", *activityMap);
//		w.write(*activityMap);
		int c = waitKey(1);
		bShouldStop = c == 27;
		if (c == 116)
			trans = !trans;
	}
	kinects[0].stopDevice();
	kinects[1].stopDevice();
	
	kinects[0].shutDown();
	kinects[1].shutDown();
	return 0;
}