#include "ActivityMap.h"
#include "BackgroundDepthSubtraction.h"
#include "Clustering.h"



void updateActivityMap(Mat& activityMap, const ActivityMap* am, const XnPoint3D* p3D, const int nP)
{
	for (int i = 0; i < nP; i++)
	{
		int xCoor = am->findCoordinate(p3D[i].X, MIN_X, MAX_X, am->xStep);
		int yC = am->findCoordinate(p3D[i].Z, MIN_Z, MAX_Z, am->depthStep);
		int yCoor = (YRes-1) - yC; //flip around X axis.

		uchar* ptr = activityMap.ptr<uchar>(yCoor);
		ptr[3*xCoor] = 0;
		ptr[3*xCoor+1] = 0;
		ptr[3*xCoor+2] = 255;

//		circle(activityMap, Point(xCoor, yCoor), 1, Scalar::all(255), 2);
	}
}


int main()
{
	ActivityMap actMapCreator(2);
	Clustering clusters;

	KinectSensor kinects[2];
	const XnDepthPixel* depthMaps[2];
	const XnRGB24Pixel* rgbMaps[2];

	kinects[0].initDevice(1, REF_CAM, true);
	kinects[1].initDevice(2, REF_CAM, true);

	kinects[0].startDevice();
	kinects[1].startDevice();

	namedWindow("Activity Map");
	Mat* activityMap;
	Mat background, whiteBack;

	//flags
	bool bShouldStop = false;
	bool trans = false;
	bool bgComplete = false;
	bool deleteBG = false;

	BackgroundDepthSubtraction subtractor1, subtractor2;
	int nFPoints1, nFPoints2;
	XnPoint3D* points2D_1 = new XnPoint3D[MAX_FORGROUND_POINTS];	
	XnPoint3D* points2D_2 = new XnPoint3D[MAX_FORGROUND_POINTS];
	XnPoint3D *p3D_1, *p3D_2;

	nFPoints1 = nFPoints2 = 0;


//	VideoWriter w ("out.avi", CV_FOURCC('M','J','P','G'), 20, actMapCreator.getResolution(), true);


	/*Mat rgbImg   (XN_VGA_Y_RES, XN_VGA_X_RES, CV_8UC3);
	Mat depthImg (XN_VGA_Y_RES, XN_VGA_X_RES, CV_8UC3);*/
	bool first = true;
	int cont = 0;
	while (!bShouldStop)
	{
		kinects[0].waitAndUpdate();
		kinects[1].waitAndUpdate();

		depthMaps[0] = kinects[0].getDepthMap();
		depthMaps[1] = kinects[1].getDepthMap();
		rgbMaps[0] = kinects[0].getRGBMap();
		rgbMaps[1] = kinects[1].getRGBMap();
		
		if (bgComplete && trans) //Trans must be true
		{
			if (first)
			{
				activityMap->copyTo(background);
				whiteBack = Mat::Mat(actMapCreator.getResolution(), CV_8UC3);
				Utils::initMat3u(whiteBack, 255);
				first = false;
			}
			else if (deleteBG)
				whiteBack.copyTo(*activityMap);
			else
				background.copyTo(*activityMap);

			nFPoints1 = subtractor1.subtraction(points2D_1, depthMaps[0]);
			nFPoints2 = subtractor2.subtraction(points2D_2, depthMaps[1]);
			if (nFPoints1 != 0 || nFPoints2 != 0)
			{
				p3D_1 = kinects[0].arrayBackProject(points2D_1, nFPoints1);
				p3D_2 = kinects[1].arrayBackProject(points2D_2, nFPoints2);
				//Transform cam 1 (RT)
				kinects[0].transformArray(p3D_1, nFPoints1);

				updateActivityMap(*activityMap, &actMapCreator, p3D_1, nFPoints1);
				updateActivityMap(*activityMap, &actMapCreator, p3D_2, nFPoints2);

				clusters.clusterImage(*activityMap);

				delete[]p3D_1;
				delete[]p3D_2;
			}
		}
		else
			activityMap = actMapCreator.createActivityMap(kinects, depthMaps, rgbMaps, trans);
		
		imshow("Activity Map", *activityMap);
		
//		w << *activityMap;
//		imwrite("out.jpg", *activityMap);
//		w.write(*activityMap);
		int c = waitKey(1);
		switch (c)
		{
		case 27: //esc
			{
				bShouldStop = true;
				break;
			}

		case 99: //c
			{
				bgComplete = true;
				break;
			}
		case 116: //t
			{
				trans = !trans;
				break;		
			}
		case 13: //enter
			{
				deleteBG = !deleteBG;
				break;
			}
		}
	}
	kinects[0].stopDevice();
	kinects[1].stopDevice();
	
	kinects[0].shutDown();
	kinects[1].shutDown();
	return 0;
}