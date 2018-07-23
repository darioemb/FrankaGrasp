#include <iostream>
#include <fstream>
#include <vector>
#include <stdlib.h>
#include <math.h>

using namespace std;

double xmin = 360, ymin = 153, w = 582, h = 364, xmax, ymax;
int iStart = 0, iEnd = 101;

void filterRect(int fnum);

int main(int argc, char* argv[]) {
	if (argc > 4) {
		xmin = atof(argv[1]);
		ymin = atof(argv[2]);
		w = atof(argv[3]);
		h = atof(argv[4]);
	}
	if (argc > 6) {
		iStart = atoi(argv[5]);
		iEnd = atoi(argv[6]);
	}
	xmax = xmin+w;
	ymax = ymin+h;
	for (int fnum = iStart; fnum <= iEnd; fnum++) {
		filterRect(fnum);
	}
}

void filterRect(int fnum) {
	char buff[200];
	sprintf(buff,"sr_%04d.txt",fnum);
	double score, x[4], y[4];
	vector<double> scores;
	vector<double> allX, allY;
	ifstream fin;
	fin.open(buff);
	while (fin >> score 
		>> x[0] >> y[0] 
		>> x[1] >> y[1] 
		>> x[2] >> y[2] 
		>> x[3] >> y[3] ) {
		bool withinRange = true;
		for (int i=0;i<4;i++) {
			if (x[i] < xmin || x[i] > xmax || y[i] < ymin || y[i] > ymax) {
				withinRange = false;
				break;
			}
		}
		if (withinRange) {
			scores.push_back(score);
			for (int i=0;i<4;i++) {
				allX.push_back(x[i]);
				allY.push_back(y[i]);
			}
		}
	}
	fin.close();
	cout << buff << ": " << scores.size() << endl;
	ofstream fout;
	fout.open(buff);
	for (unsigned int i=0;i<scores.size();i++) {
		fout << scores[i];
		for (int j=0;j<4;j++) {
			fout << " " << allX[i*4+j] << " " << allY[i*4+j]; 
		}
		fout << endl;
	}
	fout.close();
}
