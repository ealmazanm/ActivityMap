#include "ActivityMap.h"
#include "BackgroundDepthSubtraction.h"
#include "Clustering.h"

const int NUM_SENSORS = 3;

void updateActivityMap(Mat& activityMap, const ActivityMap* am, const XnPoint3D* p3D, const int nP, const XnPoint3D* points2D, const XnRGB24Pixel* rgbMap, Mat& colorMap, Mat& heightMap)
{
	for (int i = 0; i < nP; i++)
	{
		int xCoor = am->findCoordinate(p3D[i].X, MIN_X, MAX_X, am->xStep);
		int yC = am->findCoordinate(p3D[i].Z, MIN_Z, MAX_Z, am->depthStep);
		int yCoor = (YRes-1) - yC; //flip around X axis.


		XnRGB24Pixel color = rgbMap[(int)points2D[i].Y*XN_VGA_X_RES+(int)points2D[i].X];

		uchar* ptr = activityMap.ptr<uchar>(yCoor);
		uchar* ptrCM = colorMap.ptr<uchar>(yCoor);
		float* ptrHM = heightMap.ptr<float>(yCoor);

		ptrHM[xCoor] = p3D[i].Y;
		ptr[3*xCoor] = 0;
		ptr[3*xCoor+1] = 0;
		ptr[3*xCoor+2] = 255;

		ptrCM[3*xCoor] = color.nBlue;
		ptrCM[3*xCoor+1] = color.nGreen;
		ptrCM[3*xCoor+2] = color.nRed;

//		circle(activityMap, Point(xCoor, yCoor), 1, Scalar::all(255), 2);
	}
}


int main()
{
	ActivityMap actMapCreator(NUM_SENSORS);
	Clustering clusters;

	KinectSensor kinects[NUM_SENSORS];
	const XnDepthPixel* depthMaps[NUM_SENSORS];
	const XnRGB24Pixel* rgbMaps[NUM_SENSORS];

	for (int i = 0; i < NUM_SENSORS; i++)
	{
		kinects[i].initDevice(i, REF_CAM, true);
		kinects[i].startDevice();
	}

	namedWindow("Activity Map");
	Mat *activityMap, *heightMap;
	Mat background, whiteBack, colorMap;

	//flags
	bool bShouldStop = false;
	bool trans = false;
	bool bgComplete = false;
	bool deleteBG = false;

	BackgroundDepthSubtraction subtractors[NUM_SENSORS];
	int numberOfForegroundPoints[NUM_SENSORS];

	XnPoint3D* pointsFore2D [NUM_SENSORS];
	XnPoint3D* points3D[NUM_SENSORS];

	for (int i = 0; i < NUM_SENSORS; i++)
	{
		pointsFore2D[i] = new XnPoint3D[MAX_FORGROUND_POINTS];
		numberOfForegroundPoints[i] = 0;
	}


//	VideoWriter w ("out1.avi", CV_FOURCC('M','J','P','G'), 20.0, actMapCreator.getResolution(), true);


	/*Mat rgbImg   (XN_VGA_Y_RES, XN_VGA_X_RES, CV_8UC3);
	Mat depthImg (XN_VGA_Y_RES, XN_VGA_X_RES, CV_8UC3);*/
	bool first = true;
	int cont = 0;
	while (!bShouldStop)
	{
		for (int i = 0; i < NUM_SENSORS; i++)
		{
			kinects[i].waitAndUpdate();
			depthMaps[i] = kinects[i].getDepthMap();
			rgbMaps[i] = kinects[i].getRGBMap();
		}
		
		if (bgComplete && trans) //Trans must be true
		{
			//cont++;
			if (first)
			{
				//colorMap(actMapCreator.getResolution(), CV_8UC3);
				activityMap->copyTo(background);
				whiteBack = Mat::Mat(actMapCreator.getResolution(), CV_8UC3);
				Utils::initMat3u(whiteBack, 255);
				first = false;
			}
			else if (deleteBG)
				whiteBack.copyTo(*activityMap);
			else
				background.copyTo(*activityMap);

			for (int i = 0; i < NUM_SENSORS; i++)
			{
				numberOfForegroundPoints[i] = subtractors[i].subtraction(pointsFore2D[i], depthMaps[i]);
			}

//			if (nFPoints1 != 0 || nFPoints2 != 0)
			{
				whiteBack.copyTo(colorMap);
				heightMap = new Mat(actMapCreator.getResolution(), CV_32F);
				Utils::initMatf(*heightMap, -5000);

				for (int i = 0; i < NUM_SENSORS; i++)
				{
					points3D[i] = kinects[i].arrayBackProject(pointsFore2D[i], numberOfForegroundPoints[i]);
					if (kinects[i].getIdCam() != kinects[i].getIdRefCam())
						kinects[i].transformArray(points3D[i], numberOfForegroundPoints[i]);

					updateActivityMap(*activityMap, &actMapCreator, points3D[i], numberOfForegroundPoints[i], pointsFore2D[i], rgbMaps[i], colorMap, *heightMap);
				}

				imshow("Color Map", colorMap);
				clusters.clusterImage(*activityMap, &colorMap, heightMap);

				delete(heightMap);
				for (int i = 0; i < NUM_SENSORS; i++)
					delete[] points3D[i];
			}
//			w << *activityMap;
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

	for (int i = 0; i < NUM_SENSORS; i++)
	{
		delete[] pointsFore2D[i];
		kinects[i].stopDevice();
  		kinects[i].shutDown();
	}

	return 0;
}