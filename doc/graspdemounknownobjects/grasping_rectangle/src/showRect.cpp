#include "common.h"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <string>
#include <vector>
#include <math.h>
#include "feature.h"


string dirPrefix = "/misc/projects/grasping_data/11-6/";
int test(int img_num);
vector < ScoredRect > vRect;

int main(int argc, char *argv[]) {
	for (int i=0;i<102;i++) {
		test(i);
	}
	// test(101, model);
	// test(0);
	// test(145);
	return 0;
}

int test(int img_num) {
	char sarr[50];
	sprintf(sarr,"%04d",img_num);
	string filenum = sarr;
	string imgFile = (dirPrefix+"left"+filenum+".pgm").c_str();
	string rectPosFile = (dirPrefix+"left"+filenum+"_pos.txt").c_str();
	string rectFile = "sr_";
	rectFile += filenum + ".txt";
	
	vRect.clear();
	
	ifstream in; in.open(rectFile.c_str());
	ScoredRect sr; sr.rect.x.resize(4); sr.rect.y.resize(4);
	while (in >> sr.score 
		>> sr.rect.x[0] >> sr.rect.y[0] 
		>> sr.rect.x[1] >> sr.rect.y[1] 
		>> sr.rect.x[2] >> sr.rect.y[2] 
		>> sr.rect.x[3] >> sr.rect.y[3] ) {
	    vRect.push_back(sr);
	}
	in.close();
	
	IplImage *img = cvLoadImage(imgFile.c_str());
	int savePoints[] = {1,5,10,20,50,100};
	for (int i=0,j=0;j<6;j++) {
		for (; i < savePoints[j]; i++) {
			ScoredRect sr = vRect[i];
			//printf("%.4g %g %g %g %g %g\n", sr.score, sr.x, sr.y, sr.w, sr.h, sr.t);
			vector<double> x = sr.rect.x;
			vector<double> y = sr.rect.y;
			printf("%.4g", sr.score);
			for (unsigned int k = 0;k < x.size();k++) {
				printf(" %g %g", x[k], y[k]);
			}
			printf("\n");
			CvPoint  box[] = {x[0], y[0],  x[1], y[1],  x[2], y[2],  x[3], y[3]};
			CvPoint* boxArr[1] = {box};
			int      nCurvePts[1] = {4};
			int      nCurves = 1;
			int      isCurveClosed = 0;
			int      lineWidth = 1;
			cvPolyLine(img, boxArr, nCurvePts, nCurves, isCurveClosed, cvScalar(255, 0, 0), lineWidth);
		}
		char buff[50];
		sprintf(buff,"res_%s_%04d.ppm",filenum.c_str(),savePoints[j]);
		cvSaveImage(buff, img);
	}
	// cvNamedWindow("Results", CV_WINDOW_AUTOSIZE);
	// cvShowImage("Results", img);
	// cvWaitKey(0);
	cvReleaseImage(&img);
	// cvDestroyWindow("Results");
	return 0;
}
