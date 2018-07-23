/*
 * Search.h
 *
 *  Created on: Oct 19, 2010
 *      Author: Yun
 *
 *  Description: This file contains all the functions related to search. Before the search, the SVM model(s) should be ready.
 */

#ifndef SEARCH_H_
#define SEARCH_H_

#include "cv.h"
#include "highgui.h"
#include <cxcore.h>
#include "Rect.h"
#include <string>
#include <vector>
#include <queue>

using namespace std;

/*
 * The ScoredRect represents one rect and its score. It's used for sorting the rectangles by its values.
 *
 */
struct ScoredRect {
	Rect rect;
	double score;
	ScoredRect() {}
	ScoredRect(Rect r, double s) {
		rect = r;
		score = s;
	}
	/*
	 * The 'less than' is redefined for priority queue.
	 * Since we are interested in top K rects in the first step search, we use a fixed size (k) priority queue
	 * to maintain this. Thus, given k+1 rects, we need to find the one with smallest score and kick it out of
	 * queue. That's why we need the rects are sorted from the largest score to smallest.
	 */
	bool operator<(const ScoredRect &a) const{
		return score>a.score;
	}
};

/*
 * RectQueue is the priority queue stored the top k best rects.
 */
typedef priority_queue<ScoredRect> RectQueue;


/*
 * Search class provides all functions related to search
 */
class Search {
public:
	/*
	 * If the image needs to crop the left part containing the arm, set needCrop true.
	 * Default: true
	 */
	bool needCrop;
	/*
	 * the number of columns on the left side of the image you want to discard.
	 * Default: 400
	 */
	int xCrop;
	/*
	 * model1: the SVM model trained using the preliminary features
	 * model2: the SVM model trained using the advanced features
	 */
	vector<double> model1, model2;
	/*
	 * discretisized orientations of the rectangle.
	 * Default: {-90, -60, -30, 0, 30, 60}
	 */
	vector<double> angles;

	Search();
	virtual ~Search();

	/*
	 * load SVMRank model in the format of SVMlight
	 * modelFile: the file name (including the path) of the model
	 * model: the output
	 */
	void loadModel(string modelFile, vector<double> &model);
	/*
	 * Search for top k rectangles using model1, in brute-force method.
	 * folder: the path of all images
	 * testlist: the list of all images' filename.
	 * postfix: the postfix of the file storing all top rects.
	 *          For example, if the postfix='_top100.txt', then given an image 'left0000.pgm',
	 *          the top k rects will be written to file 'left0000_top100.txt' under the same
	 *          directory.
	 * nTop: how many top rects to keep.
	 * withMask: whether to use a mask to reduce the search space or not.
	 * masklist: the list of masks. They are in the order same as testlist.
	 *
	 * pre-conditions: model1 is loaded
	 */
	void search_step1_brutal(string folder, vector<string> &testlist, string postfix,
			int nTop, bool withMask, vector<string> &masklist);
	/*
	 * Search for the best rectangle using model2, and write it to file
	 * folder: the path of all images
	 * testlist: the list of all images' filename.
	 * deplist: the list of all disparity images' filename.
	 * topPostfix: the postfix of top rects. This should be as same as the 'postfix' in search_step1
	 * postfix: the postfix of the file for the final answer
	 */
	void search_step2(string folder, vector<string>&testlist, vector<string>&deplist, string topPostfix, string postfix);
private:
	/*
	 * Search for top k rectangles in one image.
	 * rawImageFile: the file name of the image.
	 * outFile: output file name to store the rects
	 * nTop: k
	 * withMask: if a mask is provided or not.
	 * maskFile: if withMask=true, this is the filename of the mask
	 */
	void search_step1_brutal(string rawImageFile, string outFile, int nTop, bool withMask, string maskFile);
	/*
	 * Search for top k rectangles in one image, given the orientation.
	 * img: the raw image. If it's using masks, then mask has already been applied on img.
	 * angle: the angle that the img is going to be rotated with.
	 * topQueue: the queue to keep top rects.
	 * nTop: k
	 */
	void search_step1_brutal(IplImage *img, double angle, RectQueue &topQueue, int nTop);
	/*
	 * Compute the combined histogram features matrix for an image.
	 * f: output.
	 *    f is a vector 3 matrices. f[0] is the weighted feature matrix of upper part, and f[1] is for
	 *    the middle part, and f[2] is for the bottom.
	 */
	void calcHist3(vector<CvMat*> &f, IplImage* img);

};


#endif /* SEARCH_H_ */
