#include <iostream>
#include <stdlib.h>
#include <fstream>

#include "opencv2/core/core_c.h"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv/highgui.h"

#include "common.h"
#include <vector>
#include "calcRectIntersection.h"
#include "testModel.h"

using namespace std;

#define APPEND_RECTS true
#define N_RECTS_TO_ADD 50
#define EVAL_TEST_SET true

bool GENERATE_NEW_TRAINING = false;
bool MODEL_PRE_TRAINED = false;
bool EARLY_STOP = false;
int N_CYCLES_START = 3;
int N_CYCLES = N_CYCLES_START + 1;

string trainingFile = "boostedTraining.txt";
string sortedTrainingFile = "sortedBoostedTraining.txt";
const char *dirPrefix = "/misc/projects/grasping_data/11-6/";
const char *labelsPrefix = "/misc/projects/grasping_data/11-6/new_labels/";
string trainVsTestFile = "train_test_split.txt";
string modelFilePrefix = "model_rank";
string statusFile = "boostStatus.txt";
string appendedRects = "appendedRects.txt";
string sortedAppendedRects = "sortedAppendedRects.txt";

void readBgMap(const char* filename, vector<int> &fNum, vector<string> &bgFilenames);

void readTrainTestSplit(const char *file, vector<int> fNum, vector<int> &trainingSet, vector<int> &testSet) {
	ifstream in; in.open(file);
	int n, isTest;
	trainingSet.clear();
	testSet.clear();
	while (in >> n >> isTest) {
		int k=0;
		while (fNum[k] != n) {
			k++;
		}
		(isTest ? testSet : trainingSet).push_back(k);
	}
}

int main(int argc, char *argv[]) {
	if (argc > 1) {
		N_CYCLES_START = atoi(argv[1]);
		N_CYCLES = N_CYCLES_START + 1;
	}
	if (argc > 2) {
		MODEL_PRE_TRAINED = atoi(argv[2]) > 0 ? true : false;
	}
	if (argc > 3) {
		GENERATE_NEW_TRAINING = atoi(argv[3]) > 0 ? true : false;
	}
	if (argc > 4) {
		EARLY_STOP = atoi(argv[4]) > 0 ? true : false;
	}

	init();
    string dir = string(dirPrefix);
	vector<int> fNum;
	vector<string> bgFilenames;
	readBgMap((dir+"background_map.txt").c_str(), fNum, bgFilenames);
	
	vector<int> trainingSet, testSet;
	readTrainTestSplit((dir+trainVsTestFile).c_str(), fNum, trainingSet, testSet);
	
    // Enumerate all rectangles
    vector<ScoredRect> allsr;
    // vector<double> all_precision;
    
	CvMat *pts = cvCreateMat(3, 4, CV_32FC1);
	for (int k = 0; k < 4; k++)
		cvmSet(pts, 2, k, 1.0);
	CvMat *rotatedPts = cvCreateMat(2, 4, CV_32FC1);
	CvMat *rotMatrix = cvCreateMat(2, 3, CV_32FC1);
    
    const int PRINT_EVERY = 100000;
    int count = 0;
    
    allsr.push_back(ScoredRect(Rect(0,20,0,20),0));
    
	for (unsigned int c = 0; c < angles.size(); c++) {
		double rad = angles[c] / 180.0 * CV_PI;
        CvSize size = cvSize(1024 * abs(cos(rad)) + 768 * abs(sin(rad)),
                        768 * abs(cos(rad)) + 1024 * abs(sin(rad)));
        cv2DRotationMatrix(cvPoint2D32f
		        (size.width / 2.0, size.height / 2.0),
		        -angles[c], 1.0, rotMatrix);
		for (unsigned int a = 0; a < xSizes.size(); a++) {
			for (unsigned int b = 0; b < ySizes.size(); b++) {
				int _dx = xSizes[a];
				int _dy = ySizes[b];
				for (int j = 0; j < size.height - _dy; j += dy) { // Rows
					for (int i = 0; i < size.width - _dx; i += dx) { // Cols
						cvmSet(pts, 0, 0, i);
						cvmSet(pts, 1, 0, j);
						cvmSet(pts, 0, 1, i);
						cvmSet(pts, 1, 1, (j + _dy));
						cvmSet(pts, 0, 2, (i + _dx));
						cvmSet(pts, 1, 2, (j + _dy));
						cvmSet(pts, 0, 3, (i + _dx));
						cvmSet(pts, 1, 3, j);

						cvGEMM(rotMatrix, pts, 1.0, NULL, 0.0, rotatedPts);
						vector<double> x, y;
						x.resize(4);
						y.resize(4);
                        bool outlier = false;
                        for (int k = 0;k < 4;k++) {
							x[k] = cvmGet(rotatedPts, 0, k) - (((float)(size.width)) / 2.0) + (1024.0 / 2.0);
							y[k] = cvmGet(rotatedPts, 1, k) - (((float)(size.height)) / 2.0) + (768.0 / 2.0);
							// cout << x[k] << "," << y[k] << " ";
							if (x[k] < 0 || y[k] < 0 || x[k] >= 1024 || y[k] >= 768) {
								outlier = true;
								break;
							}
							//y[k] = cvmGet(rotatedPts,1,k)/yScales[b];
						}
                        if (!outlier) {
                            allsr.push_back(ScoredRect(Rect(x, y), 0.0));
                            // if ((count % PRINT_EVERY) == 0) {
                                // cout << count << " " << allsr[count] << endl;
                            // }
                            count ++;
                        }
                        // all_precision.push_back(0);
					}
				}
			}
		}
	}
    
    double numTotal = 0;
    double precision = 0;
    
    for (unsigned int nFile = 0; nFile < trainingSet.size(); nFile++) {
        vector<Rect> posRects;
        int fileNum = fNum[trainingSet[nFile]];
        char posBuff[500], negBuff[500];
        // sprintf(posBuff, "%sleft%04d_pos.txt", dir.c_str(), fileNum);
        // sprintf(negBuff, "%sleft%04d_neg.txt", dir.c_str(), fileNum);
        // loadRects(rects,posBuff,negBuff);
        sprintf(posBuff, "%sleft%04d_pos.txt", labelsPrefix, fileNum);
        loadRects(posRects,posBuff);
        if (posRects.size() == 0)
            continue;
        
        vector<double> result = calcIntersection(allsr, posRects, numTotal, precision);
        if (result.size() == 0)
            continue;
        if (result.size() != allsr.size())
            cout << "ERROR: result: " << result.size() << " and allsr: " << allsr.size() << ", size differ!" << endl;
        for (unsigned int k = 0; k < result.size(); k++) {
            allsr[k].score += (result[k] >= 0.5 ? 1.0 : 0.0);
        }
    }
    sort(allsr.begin(), allsr.end());
    for (unsigned int i = 0; i < 100 && i < allsr.size(); i++) {
        cout << allsr[i] << endl;
    }
    cout << "# train numTotal: " << numTotal << ", best training precision: " << (allsr[0].score/numTotal) << endl;
    
    vector<ScoredRect> bestRect;
    bestRect.push_back(allsr[0]);
    bestRect[0].score = 0;
    numTotal = 0;
    precision = 0;
    
    for (unsigned int nFile = 0; nFile < testSet.size(); nFile++) {
        vector<Rect> posRects;
        int fileNum = fNum[testSet[nFile]];
        char posBuff[500], negBuff[500];
        // sprintf(posBuff, "%sleft%04d_pos.txt", dir.c_str(), fileNum);
        // sprintf(negBuff, "%sleft%04d_neg.txt", dir.c_str(), fileNum);
        // loadRects(rects,posBuff,negBuff);
        sprintf(posBuff, "%sleft%04d_pos.txt", labelsPrefix, fileNum);
        loadRects(posRects,posBuff);
        if (posRects.size() == 0)
            continue;
        
        vector<double> result = calcIntersection(bestRect, posRects, numTotal, precision);
        if (result.size() == 0)
            continue;
        if (result.size() != bestRect.size())
            cout << "ERROR: result: " << result.size() << " and bestRect: " << bestRect.size() << ", size differ!" << endl;
        for (unsigned int k = 0; k < result.size(); k++) {
            bestRect[k].score += (result[k] >= 0.5 ? 1.0 : 0.0);
        }
    }
    cout << "# test numTotal: " << numTotal << ", test precision: " << (bestRect[0].score/numTotal) << endl;
    cout << precision << endl;
	return 0;
}


void readBgMap(const char* filename, vector<int> &fNum, vector<string> &bgFilenames) {
    ifstream in;
    in.open(filename);
    string num, bgf;
    fNum.clear(); bgFilenames.clear();
    while (in >> num >> bgf) {
        fNum.push_back(atoi(num.c_str()));
        bgFilenames.push_back(bgf);
    }
}
