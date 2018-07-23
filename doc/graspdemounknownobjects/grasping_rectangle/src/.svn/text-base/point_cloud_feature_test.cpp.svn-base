#include "common.h"
#include <fstream>
#include "testModel.h"

// char *modelfile = "img_pc_model.mod";
// char *modelfile = "img_pc_model_poly.mod";
// char *modelfile = "img_pc_model_rbf.mod";
// char *modelfile = "img_pc_class_poly.mod";
// char *modelfile = "img_pc_class.mod";
// char *modelfile = "model_11_28_linear";
// char *modelfile = "model_11_28_fpfh_linear";
string modelfile = "model_poly_0002";
// char *modelfile = "img_pc_size_poly.mod";
//~ char *modelfile = "pc_model.mod";
// char *modelfile = "pc_model_poly.mod";
// char *modelfile = "pc_model_rbf.mod";
string dirPrefix = "/misc/projects/grasping_data/11-6/";
string img_save_dir = "/tmp/marcus/";

// const int nBins = 9;
// Used for threading
#define SAVE_IMAGES true
#define SAVE_RECTS true
#define SAVE_RECTS_N_TOP 100

void readBgMap(const char* filename, vector<int> &fNum, vector<string> &bgFilenames);

int main(int argc, char *argv[]) {
	if (argc > 1) {
		dirPrefix = argv[1];
		dirPrefix += "/";
	}
	if (argc > 2) {
		modelfile = argv[2];
	}
	cout << dirPrefix << endl;
	cout << modelfile << endl;
	//~ if (argc > 3) {
			//~ modelfile = argv[2];
	//~ }
	init();
	MODEL *model = readAndInitModel(modelfile.c_str());
	vector<int> fNum;
	vector<string> bgFilenames;
	readBgMap((dirPrefix+"background_map.txt").c_str(), fNum, bgFilenames);
	for (int i=0;i<fNum.size();i++) {
		// test(fNum[i],model,dirPrefix+bgFilenames[i]);
		cout << fNum[i] << "\t" << bgFilenames[i] << endl;
	}
	
	vector<int> savePoints;
	savePoints.push_back(1);
	savePoints.push_back(5);
	savePoints.push_back(10);
	savePoints.push_back(20);
	savePoints.push_back(50);
	savePoints.push_back(100);
	
	for (int i=0;i<fNum.size();i++) {
	  //if (fNum[i] < 40  || fNum[i] == 102) {
	  //continue;
	  //}
		if (fNum[i] <= 101 || fNum[i] >= 164) {
			CROP_RECT = cvRect(360, 153, 582, 364);
		} else {
			CROP_RECT = cvRect(400, 0, 1024-400, 768-0);
		}
		vector<ScoredRect> scoredRects = test(dirPrefix, fNum[i], model, dirPrefix+bgFilenames[i]);
		
		// Save the rectangles
		if (SAVE_RECTS) {
			char name[500];
			sprintf(name, "sr_%04d.txt",fNum[i]);
			ofstream of; of.open(name);
			for (unsigned int j=0;j<SAVE_RECTS_N_TOP && j < scoredRects.size();j++) {
				ScoredRect sr = scoredRects[j];
				of << sr.score;
				vector<double> x = sr.rect.x;
				vector<double> y = sr.rect.y;
				for (unsigned int k = 0;k < x.size();k++) {
					x[k] += CROP_RECT.x; y[k] += CROP_RECT.y;
					printf(" %g %g", x[k], y[k]);
					of << " " << x[k] << " " << y[k];
				}
				printf("\n");
				of << endl;
			}
			of.close();
		}
		if (SAVE_IMAGES) {
			char filenum[10]; sprintf(filenum,"%04d",fNum[i]);
			IplImage *img = cvLoadImage((dirPrefix+"left"+filenum+".pgm").c_str());
			for (unsigned int g=0,j=0;j<savePoints.size();j++) {
				for (; g < savePoints[j] && g < vRect.size(); g++) {
					ScoredRect sr = scoredRects[g];
					//printf("%.4g %g %g %g %g %g\n", sr.score, sr.x, sr.y, sr.w, sr.h, sr.t);
					vector<double> x = sr.rect.x;
					vector<double> y = sr.rect.y;
					for (unsigned int k = 0;k < x.size();k++) {
						x[k] += CROP_RECT.x; y[k] += CROP_RECT.y;
					}
					printf("%.4g", sr.score);
					CvPoint  box[] = {x[0], y[0],  x[1], y[1],  x[2], y[2],  x[3], y[3]};
					CvPoint* boxArr[1] = {box};
					int      nCurvePts[1] = {4};
					int      nCurves = 1;
					int      isCurveClosed = 0;
					int      lineWidth = 1;
					cvPolyLine(img, boxArr, nCurvePts, nCurves, isCurveClosed, cvScalar(0, 255, 255), lineWidth);
				}
				char buff[200];
				sprintf(buff,"%sres_%s_%04d.ppm",img_save_dir.c_str(),filenum,savePoints[j]);
				cvSaveImage(buff, img);
			}
			cvReleaseImage(&img);
		}
	}
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
