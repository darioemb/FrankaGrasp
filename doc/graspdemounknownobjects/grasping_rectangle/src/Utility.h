#pragma once

#include <algorithm>
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include "Rect.h"
#include <vector>

class Utility
{
	public:
		static IplImage* backGroundImage(const IplImage *img, const IplImage *background);
		static std::vector<double> mergeVectors(std::vector<double> v1, std::vector<double> v2);
		static IplImage* imrotate(IplImage *img, double angle);
		static Rect rectRotate(const Rect &r, double ang, int n, int m);
		static double mean(std::vector<double> array);
		static double median(std::vector<double> array);
		static IplImage* cropImage(const IplImage *img, const CvRect region);
		static void checkImage(IplImage *img);
	private:
		static int floodfill(int , int , bool[][1024], IplImage*);
		static void applyFilter(int , int , IplImage*);

};
