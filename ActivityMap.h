#pragma once
#include "KinectSensor.h"
#include "Plane.h"
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <list>
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <ctype.h>
#include "XnCppWrapper.h"

#define MAX_DEPTH 10000

using namespace std;
using namespace cv;
using namespace xn;

const int REF_CAM = 1;

//REF_CAM = 2
const int MIN_X = -4500;
const int MAX_X = 5500;
const int MIN_Z = 500;
const int MAX_Z = 5000;

const int YRes = 480;


class ActivityMap
{
public:
	ActivityMap(int);
	ActivityMap(void);
	~ActivityMap(void);

	Mat* createActivityMap(const KinectSensor* kinects, const XnDepthPixel** depthMaps, const XnRGB24Pixel** rgbMaps, bool trans);

	inline Size getResolution()
	{
		return Size(XRes, YRes);
	}

	inline int findCoordinate(float value, float minValue, float maxValue, double step) const
	{
		if (value < minValue)
			value = minValue;
		if (value > maxValue)
			value = maxValue;
		return (int)floor((value-minValue)/step);
	}

	int depthStep;
	int xStep;

private:
	XnPoint3D* convert2Points(const XnDepthPixel*);

	int XRes;

	const int NUM_SENSORS;

	int DEPTH_VAR;
	int X_VAR;

};

