#include "common.h"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <math.h>
#include "calcRectIntersection.h"

using namespace std;

// string rectDirPrefix = "/misc/projects/grasping_data/results/with_crop/";
string rectDirPrefix = "./";
string labelsDirPrefix = "/misc/projects/grasping_data/11-6/new_labels/";
string outFile = "rect_overlap.txt";
#define NUM_RECTS 1

int main(int argc, char *argv[]) {
	ofstream of;
	of.open(outFile.c_str());
	
	double numTotal = 0;
	double precision = 0;
	
	for (int i=0;i<184;i++) {
		char buff[5];
		sprintf(buff, "%04d", i);
		string posFile = labelsDirPrefix + "left" + buff + "_pos.txt";
		string rectFile = rectDirPrefix + "sr_" + buff + ".txt";
		
		// Load scored rects
		vector<ScoredRect> scoredRects;
		scoredRects.clear();
		ifstream in; in.open(rectFile.c_str());
		ScoredRect sr;
		for (int j=0;j<NUM_RECTS;j++) {
			// in >> sr.score;
			for (int k=0;k<4;k++) {
				in >> sr.rect.x[k] >> sr.rect.y[k];
			}
			in >> sr.score;
			scoredRects.push_back(sr);
		}
		in.close();
	
        if (scoredRects.size() == 0) {
            continue;
        }
		// Load positive labels
		vector<Rect> rects; rects.clear();
		loadRects(rects, posFile.c_str());
        
        if (rects.size() == 0) {
            continue;
        }
		
		vector<double> intersectionRatio = calcIntersection(scoredRects, rects, numTotal, precision);
		cout << buff;
		of << buff;
		for (int j=0;j<intersectionRatio.size();j++) {
			cout << " " << intersectionRatio[j];
			of << " " << intersectionRatio[j];
		}
		cout << endl;
		of << endl;
	}
	// calcIntersection(1);
	cout << "Precision: " << (precision/numTotal) << endl;
	of << "Precision: " << (precision/numTotal) << endl;
	of.close();
	return 0;
}
