#include "common.h"
#include <fstream>
#include "testModel.h"

string modelfile = "model";
string img_save_dir = "/misc/projects/jamming_IROS11/jam_result_img/";

bool SAVE_IMAGES = true;
#define SAVE_RECTS true
#define SAVE_RECTS_N_TOP 100

int main(int argc, char *argv[]) {
	if (argc <= 3) {
		cout << "Usage: rank imgFile bgFile pcdFile [modelFile img_save_dir]" << endl;
		return 0;
	}
	string imgFile = argv[1];
	string bgFile = argv[2];
	string pcdFile = argv[3];
	if (argc > 4) {
		modelfile = argv[4];
	}
	if (argc > 5) {
		img_save_dir = argv[5];
		SAVE_IMAGES = true;
	}
	
	init();
	CROP_RECT = cvRect(0, 0, 640, 480);
	MODEL *model = readAndInitModel(modelfile.c_str());
	
	vector<int> savePoints;
	savePoints.push_back(1);
	//~ savePoints.push_back(5);
	savePoints.push_back(10);
	//~ savePoints.push_back(20);
	//~ savePoints.push_back(50);
	//~ savePoints.push_back(100);
	
	string filenum;
	int start = imgFile.rfind('/');
	start = (start == string::npos) ? 0 : start;
	while (start < imgFile.length() && imgFile[start] < 48 || imgFile[start] > 57)
		start ++;
	if (start < imgFile.length() - 4) {
		char tmp[5];
		for (int i=0;i<4;i++)
			tmp[i] = imgFile[start+i];
		tmp[4] = 0;
		filenum = string(tmp);
	} else {
		filenum = string("0000");
	}
	
	cout << filenum << endl;
    
	if (SAVE_IMAGES) {
        BG_SAVE_FILENAME = img_save_dir + "/" + "bg_" + filenum + ".pgm";
    } else {
        BG_SAVE_FILENAME = "";
    }
	//~ return 0;
	vector<ScoredRect> scoredRects = test(imgFile.c_str(), bgFile.c_str(), pcdFile.c_str(), model);
	
	// Save the rectangles
	if (SAVE_RECTS) {
		char name[500];
		sprintf(name, "sr_jam_%s.txt",filenum.c_str());
		ofstream of; of.open(name);
		for (unsigned int j=0;j<SAVE_RECTS_N_TOP && j<scoredRects.size();j++) {
			ScoredRect sr = scoredRects[j];
			vector<double> x = sr.rect.x;
			vector<double> y = sr.rect.y;
			for (unsigned int k = 0;k < x.size();k++) {
				x[k] += CROP_RECT.x; y[k] += CROP_RECT.y;
				printf("%g %g ", x[k], y[k]);
				of << ((int)x[k]) << " " << ((int)y[k]) << " " ;
			}
			of << sr.score;
			of << endl;
			printf("%g", sr.score);
			printf("\n");
		}
		of.close();
	}
	if (SAVE_IMAGES) {
		IplImage *img = cvLoadImage(imgFile.c_str());
		for (unsigned int g=0,j=0;j<savePoints.size();j++) {
			for (; g < savePoints[j] && g < vRect.size() && g < scoredRects.size(); g++) {
				ScoredRect sr = scoredRects[g];
				//printf("%.4g %g %g %g %g %g\n", sr.score, sr.x, sr.y, sr.w, sr.h, sr.t);
				vector<double> x = sr.rect.x;
				vector<double> y = sr.rect.y;
				for (unsigned int k = 0;k < x.size();k++) {
					x[k] += CROP_RECT.x; y[k] += CROP_RECT.y;
				}
				//~ printf("%.4g", sr.score);
				CvPoint  box[] = {x[0], y[0],  x[1], y[1],  x[2], y[2],  x[3], y[3]};
				CvPoint* boxArr[1] = {box};
				int      nCurvePts[1] = {4};
				int      nCurves = 1;
				int      isCurveClosed = 0;
				int      lineWidth = 1;
				cvPolyLine(img, boxArr, nCurvePts, nCurves, isCurveClosed, cvScalar(0, 255, 255), lineWidth);
			}
			char buff[200];
			sprintf(buff,"%sres_%s_%04d.ppm",img_save_dir.c_str(),filenum.c_str(),savePoints[j]);
			cvSaveImage(buff, img);
		}
		cvReleaseImage(&img);
	}
	return 0;
}
