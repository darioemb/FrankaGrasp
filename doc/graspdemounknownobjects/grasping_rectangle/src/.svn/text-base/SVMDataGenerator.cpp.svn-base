#include "SVMDataGenerator.h"

#include <cmath>
#include <dirent.h>
#include "cv.h"
#include "cxcore.h"
#include "highgui.h"
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <algorithm>
#include "Rect.h"
#include "Utility.h"

IplImage* rawImage;
IplImage* depImage;


vector<CvMat*> SVMDataGenerator::filters;
bool SVMDataGenerator::usingDep = true;

SVMDataGenerator::SVMDataGenerator(void) :
			nFilters(17) {
	needCrop = true;
	xCrop = 400;
	featureMatrix.clear();
}

SVMDataGenerator::~SVMDataGenerator(void) {
}

void SVMDataGenerator::getFileList(string dir, vector<string> &list, vector<
		string> &deplist) {
	DIR *dp;
	struct dirent *dirp;

	list.clear();
	deplist.clear();
	if ((dp = opendir(dir.c_str())) == NULL) {
		cout << "Error: opening directory " << dir << endl;
		return;
	}
	while ((dirp = readdir(dp)) != NULL) {
		string name = string(dirp->d_name);
		if (name.find("left") == 0 && name.find("pgm") != string::npos) {
			list.push_back(name);
		} else if (name.find("frame") == 0)
			deplist.push_back(name);
	}
	sort(list.begin(), list.end());
	sort(deplist.begin(), deplist.end());
}
void SVMDataGenerator::initFilters() {
	int n, m;
	double x;

	filters.resize(17);
	ifstream fin("filters.txt");
	for (int i = 0; i < 17; i++) {
		fin >> n >> m;
		filters[i] = cvCreateMat(n, m, CV_32FC1);
		for (int j = 0; j < n; j++)
			for (int k = 0; k < m; k++) {
				fin >> x;
				cvmSet(filters[i], j, k, x);
			}
	}
}

CvMat* SVMDataGenerator::getFilter(int idx) {
	if (filters.empty())
		initFilters();

	return idx < (int)filters.size() ? filters[idx] : NULL;
}

void SVMDataGenerator::calculateFilterBanks17(IplImage* img, vector<CvMat*>& H) {

	IplImage *ycbcrImg, *yImg, *cb, *cr;

	ycbcrImg = cvCreateImage(cvSize(img->width, img->height), img->depth,img->nChannels);
	yImg = cvCreateImage(cvSize(img->width, img->height), img->depth, 1);
	cb = cvCreateImage(cvSize(img->width, img->height), img->depth, 1);
	cr = cvCreateImage(cvSize(img->width, img->height), img->depth, 1);
	cvCvtColor(img, ycbcrImg, CV_RGB2YCrCb);
	cvSplit(ycbcrImg, yImg, cb, cr, 0);

	H.resize(17);
	for (int i = 0; i < (int)H.size(); i++) {
		H[i] = cvCreateMat(img->height, img->width, CV_32FC1);

		if (i == 10){
			cvFilter2D(cb, H[i], getFilter(i));
		}else if (i == 11){
			cvFilter2D(cr, H[i], getFilter(i));
	    }else{
			cvFilter2D(yImg, H[i], getFilter(i));
	    }

		//normalize them
//		double minv, maxv;
//		cvMinMaxLoc(H[i], &minv, &maxv);
		cvNormalize(H[i], H[i], 0, 1, CV_MINMAX);
	}
	cvReleaseImage(&ycbcrImg);
	cvReleaseImage(&yImg);
	cvReleaseImage(&cb);
	cvReleaseImage(&cr);
}

void SVMDataGenerator::getFeatureVector(Rect rect, vector<double>& fv1, vector<double>& fv2) {

	double ang = atan2(rect.y[3] - rect.y[0], rect.x[3] - rect.x[0]);
	//ang = ang/acos(-1.0) * 180;
	IplImage* img;
	IplImage* dep;
	/*rotate the image & disparity image*/
	img = Utility::imrotate(rawImage, ang/acos(-1.0) * 180);
	int n = img->height;
	int m = img->width;

	if (usingDep) {
		dep = Utility::imrotate(depImage, ang / acos(-1.0) * 180);
	}

	rect = Utility::rectRotate(rect, ang, n, m);

	calculateFilterBanks17(img, featureMatrix);
    //printf("get feature vector");
	getFeatureVectorRotatedImage(dep, featureMatrix, rect, fv1, fv2);

	cvReleaseImage(&img);
	if(usingDep) cvReleaseImage(&dep);
	for (int i = 0; i < featureMatrix.size(); i++)
		cvReleaseMat(&featureMatrix[i]);
}

void SVMDataGenerator::getFeatureVectorRotatedImage(IplImage* depImg, vector<CvMat*> &featureMatrix,
		Rect rect, vector<double>& fv1, vector<double>& fv2) {
    printf("calculating feature vector \n");
	/*
	 * The rectangle has the range of [minrow, maxrow]x[mincol,maxcol]
	 */
	int minrow = min(rect.y[0], rect.y[1]);
	int maxrow = max(rect.y[0], rect.y[1]);
	int mincol = min(rect.x[0], rect.x[3]);
	int maxcol = max(rect.x[0], rect.x[3]);
	int bound = (maxrow - minrow) / 3;

	//printf("bound = %d \n",bound);
	/*
	 * Compute the first feature vector, based on raw image
	 */
	fv1.clear();
	fv1.resize(15 * featureMatrix.size(), 0);
	for (int i = 0; i < (int)featureMatrix.size(); i++) {
		CvMat* h = featureMatrix[i];
		for (int j = minrow; j <= maxrow; j++) {
			const float* ptr = (const float*) (h->data.ptr + j * h->step);
			for (int k = mincol; k <= maxcol; k++) {
				int idx = min(4, int(*(ptr + k) * 5));
				if (j < minrow + bound)
					fv1[i * 15 + idx]++;
				else if (j < minrow + bound * 2)
					fv1[i * 15 + idx + 5]++;
				else
					fv1[i * 15 + idx + 10]++;
			}
		}
	}

	fv2.clear();
	return;
	/*
	 * Compute the 2nd feature vector, based on both raw and disparity image.
	 */
	uchar* data = (uchar*) depImg->imageData;
	int step = depImg->widthStep;
//	int n = depImg->height;
//	int m = depImg->width;
//	int p = depImg->depth;
	int nChannels = depImg->nChannels;
//	bool usingDep = true;

	fv2.clear();
	if (usingDep) {
		fv2.resize(2 * fv1.size() + 12,0);
	} else {
		fv2.resize(2*fv1.size(),0);
	}

	for (int count = 0; count < (int)fv1.size(); count++) {

		fv2[count] = (fv1[count] + 1) / ((maxrow-minrow+1)*(maxcol-mincol+1));
	}

	int cnt = fv1.size() - 1;

	for (int k = 0; k < featureMatrix.size() - 1; k++) {
		for (int i = 0; i < 5; i++) {
			fv2[cnt + 1] = fv2[k * 15 + i] * fv2[k * 15 + i + 10];
			fv2[cnt + 2] = fv2[k * 15 + i] / fv2[k * 15 + i + 5];
			fv2[cnt + 3] = fv2[k * 15 + i + 10] / fv2[k * 15 + i + 5];
			cnt = cnt + 3;
		}
	}

	std::vector<double> d1;
	std::vector<double> d2;
	std::vector<double> d3;

	if (usingDep) {

		//printf("\n rect: %d %d %d %d",minrow,mincol,maxrow,maxcol);

		//CvRect r = cvRect(mincol, minrow, maxcol-mincol, maxrow-minrow);
		//cvSetImageROI(depImg, r);
		//Utility::checkImage(depImg);

		for (int i = minrow; i <= maxrow; i++) {
			for (int j = mincol; j <=maxcol; j++) {
					int index = i*step+j*nChannels;
					//cout << "index = " << index << endl;
					//cout << "data[index] = " << (int)data[index] << endl;
					if ((int)data[index] <= 35) {

						if (i <= minrow+bound) {
							fv2[cnt + 1] = fv2[cnt + 1] + 1;
						} else if (i <= maxrow - bound) {
							fv2[cnt + 3] = fv2[cnt + 3] + 1;
						} else {
							fv2[cnt + 5] = fv2[cnt + 5] + 1;
						}
					} else if ((int)data[index] > 230) {

						if (i <= minrow+bound) {
							fv2[cnt + 2] = fv2[cnt + 2] + 1;
						} else if (i <= maxrow - bound) {
							fv2[cnt + 4] = fv2[cnt + 4] + 1;
						} else {
							fv2[cnt + 6] = fv2[cnt + 6] + 1;
						}

					} else {
                      //  printf("i, j = %d %d \n",i,j);
                      //  printf("minrow, bound = %d %d \n",minrow, bound);

						if (i <= minrow+bound) {
							d1.push_back((double)data[index]);
						} else if (i <= maxrow - bound) {
							d2.push_back((double)data[index]);
						} else {
							d3.push_back((double)data[index]);
						}
					}
			}
		}


		for (int i = cnt; i < cnt + 6; cnt++) {

			fv2[i] = fv2[i] / ((maxrow-minrow+1)*(maxcol-mincol+1));
		}

		if (d1.empty()) {
			d1.push_back(1);
		}
		if (d2.empty()) {
			d2.push_back(1);
		}
		if (d3.empty()) {
			d3.push_back(1);
		}
		double mean_d1 =Utility::mean(d1);
		double mean_d2 = Utility::mean(d2);
		double mean_d3 = Utility::mean(d3);
		double median_d1= Utility::median(d1);
		double median_d2= Utility::median(d2);
		double median_d3= Utility::median(d3);

		fv2[cnt + 7] =  mean_d1/mean_d2;
		fv2[cnt + 8] = mean_d3 /mean_d2;
		fv2[cnt + 9] = fv2[cnt + 7] * fv2[cnt + 8];
		fv2[cnt + 10] = median_d1/ median_d2;
		fv2[cnt + 11] = median_d3/ median_d2;
		fv2[cnt + 12] = fv2[cnt + 10] * fv2[cnt + 11];

	}
}

void SVMDataGenerator::generateSVMData(string trainDir, vector<string>&trainList, vector<string>&trainDepList,
		string outFile1, string outFile2) {
	FILE *fid1 = fopen(outFile1.c_str(), "w");
	FILE *fid2 = fopen(outFile2.c_str(), "w");

	Rect posRect, negRect;
    IplImage* temp;
	// for each image in the directory
   //trainList.size()
	for (int i = 0; i < trainList.size(); i++) {

		string rawImagePath = trainDir + trainList[i];
		string depImagePath = trainDir + trainDepList[i];

		string prefix = trainList[i].substr(0, trainList[i].length() - 4);
		//	cout<<rawImagePath<<' '<<depImagePath<<endl;

		rawImage = cvLoadImage(rawImagePath.c_str(), 3);
		if (usingDep) {
			depImage = cvLoadImage(depImagePath.c_str(), 3);
		}
		cout << "prefix: "<<prefix << endl;
		vector<Rect> rects;
		loadRects(rects, trainDir + prefix + "_pos.txt", trainDir + prefix
				+ "_neg.txt");

		if (rects.empty()) {
			cout << "WARNING: no neg file for " << prefix << endl;
			continue;
		}
		//cout << "#rectangles = " << rects.size() << endl;

		if (needCrop) {
			// crop the image
			temp = Utility::cropImage(rawImage, cvRect(xCrop, 0, rawImage->width
					- xCrop, rawImage->height));
			cvReleaseImage(&rawImage);
			rawImage = temp;
			//cout << "after croping the image " << endl;
			if (usingDep) {
				temp = Utility::cropImage(depImage, cvRect(xCrop, 0, depImage->width
						- xCrop, depImage->height));
				cvReleaseImage(&depImage);
				depImage = temp;
			}
			//cout << "after croping the dep " << endl;
			for (int k = 0; k < rects.size(); k++) {
				for (int j = 0; j < 4; j++)
					rects[k].x[j] = max(0.0, rects[k].x[j] - xCrop);
			}
		}

		vector<double> f1;
		vector<double> f2;

		for (int j = 0; j < rects.size(); j++) {

			getFeatureVector(rects[j], f1, f2);

			fprintf(fid1, "%d qid:%d", rects[j].isPos ? 2 : 1, i + 1);
			fprintf(fid2, "%d qid:%d", rects[j].isPos ? 2 : 1, i + 1);

			for (int k = 0; k < f1.size(); k++) {
				fprintf(fid1, " %d:%.3lf", k + 1, f1[k]);
			}
			//cout << "#f2 " << f2.size() << endl;
			for (int k = 0; k < f2.size(); k++) {
				fprintf(fid2, " %d:%.3lf", k + 1, f2[k]);
			}
			fprintf(fid1, "\n");
			fprintf(fid2, "\n");
		}
		cvReleaseImage(&rawImage);
		if (usingDep) cvReleaseImage(&depImage);
	}
	fclose(fid1);
	fclose(fid2);
}

void SVMDataGenerator::loadRects(vector<Rect> &rects, string posFile,
		string negFile) {
	Rect r;
	rects.clear();
	// load posRect from file
	ifstream fin(posFile.c_str());
	fin >> r;
	r.isPos = true;
	rects.push_back(r);
	fin.close();

	fin.open(negFile.c_str(), ifstream::in);
	while (fin >> r) {
		r.isPos = false;
		rects.push_back(r);
	}
	fin.close();
}
