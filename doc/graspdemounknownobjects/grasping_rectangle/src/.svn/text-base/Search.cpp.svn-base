/*
 * Search.cpp
 *
 *  Created on: Oct 19, 2010
 *      Author: helen
 */
#include "Search.h"
#include "Rect.h"
#include <algorithm>
#include <cstdio>
#include <fstream>
#include "Utility.h"
#include "SVMDataGenerator.h"

using namespace std;

Search::Search() {
	double ang[] = {-90, -60, -30, 0, 30, 60};
	angles = vector<double>(ang, ang+6);
	needCrop = true;
	xCrop = 400;
}

Search::~Search() {
	// TODO Auto-generated destructor stub
}

void Search::loadModel(string modelFile, vector<double> &model) {
//	FILE *f = fopen(modelFile.c_str(), "r");
	ifstream fin(modelFile.c_str());
	char s[10000];
	for (int i=0; i<7; i++)
		fin.getline(s,10000);
	int nfeature, nsv;
	double b;
	fin>>nfeature;  //highest feature index
	fin.getline(s,10000);
	fin.getline(s,10000);  //number of training documents
	fin>>nsv;  //number of support vectors plus 1
	fin.getline(s,10000);
	fin>>b;  //threshold b, each following line is a SV (starting with alpha*y)
	fin.getline(s,10000);
	model.clear();
	model.resize(nfeature-1,0);
	for (int i=0; i<nsv; i++) {
		double ay, x;
		int id;
		char ch;
		fin>>ay;
		while (fin>>id) {
			fin>>ch>>x;
			model[id-1]+=ay*x;
		}
	}
	fin.close();
}

void Search::search_step1_brutal(string folder, vector<string>&testlist, string postfix, int nTop,
		bool withMask, vector<string>&masklist) {
	for (vector<string>::iterator it = testlist.begin(); it!=testlist.end(); it++) {
//		printf("%s\n", it->c_str());
		string outfile = it->substr(0, it->length() - 4) + postfix;
//		printf("%s\n", outfile.c_str());
		search_step1_brutal(folder+*it, folder + outfile, nTop, withMask, string(""));
	}
}

/**
 *
 */
void Search::search_step1_brutal(string imageFile, string outFile, int nTop, bool withMask, string maskFile) {
	printf("%s %s\n", imageFile.c_str(), outFile.c_str());
	IplImage* img = cvLoadImage(imageFile.c_str(), 3);
	if (needCrop)
		img = Utility::cropImage(img, cvRect(xCrop, 0, img->width - xCrop, img->height));
	RectQueue top;
	for (vector<double>::iterator it = angles.begin(); it!=angles.end(); it++) {
		printf("rects size: %d\n", top.size());
		search_step1_brutal(img, *it, top, nTop);
		ScoredRect r = top.top();
		for (int j=0; j<4; j++)
			printf("x0=%.2lf y0=%.2lf x2=%.2lf y2=%.2lf\n", r.rect.x[0], r.rect.y[0], r.rect.x[2], r.rect.y[2]);
	}
	cvReleaseImage(&img);
	FILE *f = fopen(outFile.c_str(), "w");
	for (int i=(int)top.size(); i>0; i--) {
		ScoredRect r = top.top();
		if (needCrop) {
			for (int j=0; j<4; j++)
				r.rect.x[j]+=xCrop;
		}
		top.pop();
		for (int j=0; j<4; j++)
			fprintf(f, "%.2lf %.2lf\n", r.rect.x[j], r.rect.y[j]);
	}
	fclose(f);
}

void Search::calcHist3(vector<CvMat*> &f, IplImage* img) {
	vector<CvMat*> featureMatrix;
	SVMDataGenerator::calculateFilterBanks17(img, featureMatrix);
//	printf("calcfilter17\n");
	f.resize(3);
	int n = img->height;
	int m = img->width;
	for (int i=0; i<3; i++) {
		f[i] = cvCreateMat(n,m,CV_32FC1);
		cvSetZero(f[i]);
	}
	const float* ptr;
	float *fptr[3], *lastrow[3];
	for (int k=0; k<(int)featureMatrix.size(); k++)
		for (int i=0; i<n; i++) {
			ptr = (const float*)(featureMatrix[k]->data.ptr + featureMatrix[k]->step *i);
			if (i)
				for (int j=0; j<3; j++)
					lastrow[j] = fptr[j];
			for (int j=0; j<3; j++)
				fptr[j] = (float*)(f[j]->data.ptr + f[j]->step*i);
			for (int j=0; j<m; j++) {
				int idx = min((int)ptr[j]*5, 4);
				fptr[0][j]=model1[k*15+idx];
				fptr[1][j]=model1[k*15+idx+5];
				fptr[2][j]=model1[k*15+idx+10];
				if (i>0) {
					fptr[0][j]+=lastrow[0][j];
					fptr[1][j]+=lastrow[1][j];
					fptr[2][j]+=lastrow[2][j];
				}
			}
		}
	for (int i=0; i<featureMatrix.size(); i++)
		cvReleaseMat(&(featureMatrix[i]));
}
void Search::search_step1_brutal(IplImage *img, double angle, RectQueue &top, int nTop) {
	vector<CvMat*> f;
	IplImage* newimg = Utility::imrotate(img, angle);
//	checkImage(newimg);

//	printf("calcHist3 began\n");
	calcHist3(f, newimg);
//	printf("calcHist3 done\n");
	cvReleaseImage(&newimg);
	int n = img->height;
	int m = img->width;
//	printf("n=%d m=%d\n", n, m);
	float* fptr[3][2];
	for (int i1=n/5; i1<=n*4/5; i1++)
		for (int i2=i1+20; i2<=i1+100 && i2<=n; i2+=6) {
			int bound = (i2-i1+1)/3;
			for (int i=0; i<3; i++) {
				fptr[i][0] = (float*) (f[i]->data.ptr + f[i]->step * (i1+ bound*i -1));
				fptr[i][1] = (float*)(f[i]->data.ptr + f[i]->step * (i1+ bound*i -1 + bound));
			}
			for (int j1=m/5; j1<=m*4/5; j1+=5) {
				double score = 0;
				for (int j2 = j1; j2<=j1+100 && j2<=m*4/5; j2++) {
					for (int i=0; i<3; i++)
						score+=fptr[i][1][j2] - fptr[i][0][j2];
//					printf("score = %f\n", score);
					if (top.empty() || score>top.top().score) {
						top.push(ScoredRect(Utility::rectRotate(Rect(i1, i2, j1, j2), -angle, n, m), score));
//						printf("size %d\n", top.size());
						if (top.size()>nTop)
							top.pop();
					}
				}
			}
		}
	for (int i=0; i<3; i++)
		cvReleaseMat(&f[i]);
}

//TODO
void Search::search_step2(string folder, vector<string>&testlist, vector<string>&deplist, string topPostfix, string postfix) {
}


