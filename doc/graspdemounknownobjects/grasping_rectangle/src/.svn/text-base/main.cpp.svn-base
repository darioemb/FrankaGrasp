#include "Rect.h"
#include "SVMDataGenerator.h"
#include "Search.h"
#include <string>
#include <vector>
#include <cstdio>

using namespace std;

void test(){
	IplImage *img = cvLoadImage("data/left0003.pgm", 3);
	printf("%d %d\n", img->width, img->height);
	IplImage *img1 = cvLoadImage("data/left0003.pgm", 3);
	printf("%d %d\n", img1->width, img1->height);
	IplImage *img2 = cvLoadImage("data/left0003.pgm", 3);
	printf("%d %d\n", img2->width, img2->height);

}
int main(){

	SVMDataGenerator svm;

	SVMDataGenerator::filters.clear();
	svm.trainDir = "data/";

	svm.outFile1 = "train1.txt";
	svm.outFile2 = "train2.txt";

	svm.getFileList(svm.trainDir, svm.trainList, svm.trainDepList);
//	svm.generateSVMData();

	Search search;
	string model = "all_model_1.txt";
	search.loadModel(model, search.model1);

	printf("dataset : %d\n", svm.trainList.size());
	vector<string> maskList;
	search.search_step1_brutal(svm.trainDir, svm.trainList, "_top10.txt", 10, false, maskList);
	return 0;

}
