#pragma once
#include <vector>
#include <string>
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include "Rect.h"
#include "Utility.h"

using namespace std;

typedef vector<vector<vector<double> > > Matrix3D;

class SVMDataGenerator
{
public:
	/*
	 * the raw image
	 */
	IplImage* rawImage;
	/*
	 * the disparity image
	 */
	IplImage* depImage;

    /*
     * a vector of matrices after applied different filters (currently it's 17 features).
     */
    vector<CvMat*> featureMatrix;
    /*
     * list of filters
     */
    static vector<CvMat*> filters;

	/*
	 * If the image needs to crop the left part containing the arm, set needCrop true.
	 * Default: true
	 */
    bool needCrop;
    /*
     * whether to use disparity image
     */
    static bool usingDep;
    /*
     * number of filters
     */
    int nFilters;
	/*
	 * the number of columns on the left side of the image you want to discard.
	 * Default: 400
	 */
    int xCrop;

	SVMDataGenerator(void);
	~SVMDataGenerator(void);
	/*
	 * get the list of raw images and disparity images
	 * dir: the folder's path
	 * list: list of raw images (filenames)
	 * deplist: list of disparity images
	 */
	void getFileList(string dir, vector<string> &list, vector<string> &deplist);
	/*
	 * generate data for SVM training.
	 * pre-conditions: trainDir, trainDepList, trainList, outFile1, outFile2 should be specified already.
	 */
	void generateSVMData(string trainDir, vector<string>&trainList, vector<string>&trainDepList,
			string outFile1, string outFile2);
	/*
	 * compute matrices of applying filters on img.
	 * img: the raw image
	 * featureMatrix: vector of matrices of the filters.
	 */
	static void calculateFilterBanks17(IplImage* img, vector<CvMat*> &featureMatrix);
	static void getFeatureVectorRotatedImage(IplImage* depImg, vector<CvMat*> &featureMatrix, Rect rect, vector<double>& fv1, vector<double>& fv2);

private:
	void SVMRankData4_2steps(vector<string> trainList, vector<string> trainDepList, string trainFile1, string trainFile2);
    void getFeatureVector(Rect rect, std::vector<double>& f1, std::vector<double>& f2);
	void loadRects(vector<Rect> &rects, string posFile, string negFile);
	static CvMat* getFilter(int idx);
	static void initFilters();
};

