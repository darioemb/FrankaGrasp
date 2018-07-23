#include "common.h"
#include "feature.h"
#include "extractTrainingData.h"
#include "calcRectIntersection.h"
#include <fstream>

using namespace std;

char *dirPrefix = "/misc/projects/jamming_IROS11/"; //"/misc/projects/grasping_data/11-6/";

int main(int argc, char *argv[])
{
	if (argc!=2)
	{
		printf("Usage: %s outputFilename\n", argv[0]);
		return 0;
	}
  string dir = string(dirPrefix);
  vector<string> files = vector<string>();
  getdir(dir,files);

  ofstream fout(argv[1]);
	
  for (int nFile = 72; nFile < files.size(); nFile++) {
  // for (int nFile = 1; nFile < 2; nFile++) {
		vector<Rect> rects;
		vector<Rect> posRects;
		/*===========changed by Yun===================*/
		int fileNum = atoi(files[nFile].substr(3,4).c_str());
		loadRects(rects,(dir+files[nFile].substr(0,8)+"_jampos.txt").c_str(),(dir+files[nFile].substr(0,8)+"_jamneg.txt").c_str());
		loadRects(posRects,(dir+files[nFile].substr(0,8)+"_jampos.txt").c_str());
		cout << nFile << "\t"<< files[nFile].substr(3,4) << ", #rect= " << rects.size() << endl;
		vector< vector<float> > all_fv = extractTrainingData(dir, fileNum, rects);
		cout<<"all_fv size: "<<all_fv.size()<<endl;
		double numTotal = 0, precision = 0;
		//vector<double> evalScores = calcIntersection(rects, posRects, numTotal, precision);
		for (unsigned int k=0;k<all_fv.size();k++) {
			//cout << evalScores[k] << " qid:" << (fileNum+1);
			fout << (rects[k].isPos?"+1":"-1") << " qid:" << (fileNum+1);
			for (unsigned int i=0;i<all_fv[k].size();i++) {
				if (i>0 && all_fv[k][i] == 0) continue;
				fout << " " << (i+1) << ":" << all_fv[k][i];
			}
			fout<<endl;
		}
  }
  return 0;
  }
