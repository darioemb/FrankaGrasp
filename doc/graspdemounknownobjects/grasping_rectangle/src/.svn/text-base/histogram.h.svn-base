#pragma once
#ifndef HISTOGRAM_H
#define HISTOGRAM_H
#include "opencv2/core/core_c.h"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
#include <vector>
#include <stdio.h>
#define HISTOGRAM_ROWS 3
#define HISTOGRAM_COLS 1

// N_BINS is the number of bins about the mean to extract.  MUST BE ODD!

using namespace std;

typedef struct IntegralHistogram {
    int w,h;    // w = # cols, h = # rows
    int nBins;
    float *img; 
    // img[ (row*w + col) * nBins + k] = 
    // # of pixel(r,c) such that r <= row, c <= col that belong to bin(k)
    // This is an integram image, used to speed up histogram lookups.
    float *ranges; // k+1 floats to specify the lower/upper limits of the bins
} IntegralHistogram;

// Gets the value of the h->img at row r, col c, bin k
inline int get(const IntegralHistogram *h, int r, int c, int k);

// Gets the histogram of the rectangle with r <= row < r+h, c <= col < c+w
// O(1) operation
void getHistogramRect(const IntegralHistogram *hist, int r, int c, int h, int w, vector<float> &results);

void releaseIntegralHistogram(IntegralHistogram **hist);

// mat must be in CV32FC1
// O( width * height * nBins ) time
IntegralHistogram *calcIntegralHistogram(const CvMat *mat, int nBins, const float *ranges, const CvMat *mask);

// Get the mean bin
int meanBin(const vector<float> hist);

// Get the highest bin
int modeBin(const vector<float> hist);

// Gets the vector of bins that are between target +- width inclusive
// Eg. target = 4, width = 1, returns bins 3,4,5
void getRange(const vector<float> hist, int target, int width, vector<float> &results);

// Gets the total frequency in the histogram
float getCount(const vector<float> hist);

// Normalize so that results are not dependent on density of point cloud in that region
void normalize(vector<float> &hist);

// Get the length 27 feature vector for a given rect
void getHistogramFeatureVector(const IntegralHistogram *intHist, int nBins, int r, int c, int h, int w, vector<float> &results, bool useMean = true);

// Used for getting the feature vector for only once
void getHistogramFeatureVectorDirect(const CvMat *mat, int nBins, float binWidth, const CvMat *mask, vector<float> &results, bool useMean = true);

vector<float> calcNonlinearFeatures(vector<float> hist);

inline float getCvIntegralRect(const CvMat *mat, int r, int c, int h, int w) {
  float fsum = cvGet2D(mat, r+h, c+w).val[0];
  fsum += cvGet2D(mat, r, c).val[0];
  fsum -= cvGet2D(mat, r+h, c).val[0];
  fsum -= cvGet2D(mat, r, c+w).val[0];
  fsum /= w * h;
  return fsum;
}

#endif // HISTOGRAM_H
