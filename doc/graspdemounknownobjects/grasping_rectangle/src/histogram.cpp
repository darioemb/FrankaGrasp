#include "histogram.h"
#include <iostream>

// Gets the value of the h->img at row r, col c, bin k
inline int get(const IntegralHistogram *h, int r, int c, int k) {
    if (r < 0 || c < 0) return 0;
    if (r >= h->h) r = h->h-1;
    if (c >= h->w) c = h->w-1;
    return h->img[ (r*h->w + c) * h->nBins + k ];
}

// Gets the histogram of the rectangle with r <= row <= r+h, c= < col <= c+w
// O(1) operation
void getHistogramRect(const IntegralHistogram *hist, int r, int c, int h, int w, vector<float> &results) {
    results.resize(hist->nBins);
    //int *val = new int[hist->nBins];
    for (int i=0;i<hist->nBins;i++) {
        results[i] = get(hist,r+h-1,c+w-1,i) - get(hist,r+h-1,c-1,i) - get(hist,r-1,c+w-1,i) + get(hist,r-1,c-1,i);
    }
    //return val;
}

void releaseIntegralHistogram(IntegralHistogram **hist) {
    IntegralHistogram *h = (*hist);
    delete[] (h->ranges);
    delete[] h->img;
    delete h;
    (*hist) = NULL;
    return;
}

// mat must be in CV32FC1
// O( width * height * nBins ) time
IntegralHistogram *calcIntegralHistogram(const CvMat *mat, int nBins, const float *ranges, const CvMat *mask) {
    CvScalar s;
    IntegralHistogram *h = new IntegralHistogram();
    h->w = mat->width;
    h->h = mat->height;
    h->nBins = nBins;
    h->ranges = new float[h->nBins+1];
    memcpy(h->ranges,ranges,sizeof(float)*(h->nBins+1));
    h->img = new float[ h->h * h->w * h->nBins ];
    memset(h->img, 0, (h->h * h->w * h->nBins) * sizeof (float));
    
    float centers[h->nBins];
    for (int i=0;i<h->nBins;i++) {
        centers[i] = (ranges[i] + ranges[i+1] ) / 2.0;
    }
    
    for (int a=0;a<h->h;a++) { // rows
        for (int b=0;b<h->w;b++) { // cols
            int c = 0;
            if (mask == NULL || (cvGet2D(mask,a,b)).val[0] != 0) {
                float f = cvmGet(mat,a,b);
                if (f<ranges[0] || f > ranges[h->nBins] * 1.0001) {
                    // for (c=0;c<h->nBins;c++)
                        // h->img[ (a*h->w + b) * h->nBins + c ] = 0;
                }
                else {
                    // bool isFound = false;
                    if (f <= centers[0]) {
                        h->img[ (a*h->w + b) * h->nBins + 0 ] = 1;
                        // isFound = true;
                    } else {
                        // h->img[ (a*h->w + b) * h->nBins + 0 ] = 0;
                    }
                    // Process the first n-1 bins
                    for (c=1;c<h->nBins;c++) {
                        if (centers[c-1] < f && f <= centers[c]) {
                            float ratio = (f-centers[c-1])/(centers[c] - centers[c-1]);
                            if (ratio < 0) {
                                printf("ERR! RATIO NEGATIVE!\n");
                            }
                            h->img[ (a*h->w + b) * h->nBins + c - 1 ] = 1.0 - ratio;
                            h->img[ (a*h->w + b) * h->nBins + c ] = ratio;
                            // isFound = true;
                        } else {
                            // h->img[ (a*h->w + b) * h->nBins + c ] = 0;
                        }
                    }
                    // for (c=0;c<h->nBins-1;c++) {
                        // if (!isFound && f < ranges[c+1]) {
                            // h->img[ (a*h->w + b) * h->nBins + c ] = 1;
                            // isFound = true;
                        // } else {
                            // h->img[ (a*h->w + b) * h->nBins + c ] = 0;
                        // }
                    // }
                    // If we haven't found it by now, the n'th bin must contain it
                    // if (!isFound)
                    if (f > centers[h->nBins-1])
                        h->img[ (a*h->w + b) * h->nBins + h->nBins - 1 ] = 1;
                    // else
                        // h->img[ (a*h->w + b) * h->nBins + h->nBins - 1 ] = 0;
                }
            } else {
                // for (c=0;c<h->nBins;c++)
                    // h->img[ (a*h->w + b) * h->nBins + c ] = 0;
            }
            // Calc integral
            if (a > 0) {
                for (c=0;c<h->nBins;c++)
                    h->img[ (a*h->w + b) * h->nBins + c ] += 
                        h->img[ ((a-1)*h->w + b) * h->nBins + c ];
            }
            if (b > 0) {
                for (c=0;c<h->nBins;c++)
                    h->img[ (a*h->w + b) * h->nBins + c ] += 
                        h->img[ (a*h->w + (b-1)) * h->nBins + c ];
            }
            if (a > 0 && b > 0) {
                for (c=0;c<h->nBins;c++)
                    h->img[ (a*h->w + b) * h->nBins + c ] -= 
                        h->img[ ((a-1)*h->w + (b-1)) * h->nBins + c ];
            }
        }
    }
    // Check integral histogram
    // for (int a=0;a<h->h;a++) { // rows
        // for (int b=0;b<h->w;b++) { // cols
            // for (int c=0;c<h->nBins;c++) {
                // cout << h->img[ (a*h->w + b) * h->nBins + c ] << " ";
            // }
            // cout << endl;
        // }
        // cout << endl;
    // }
    return h;
}

// Gets the total frequency in the histogram
float getCount(const vector<float> hist) {
    float sum = 0.0;
    for (int i=0;i<hist.size();i++)
        sum += hist[i];
    return sum;
}

// Get the mean bin
int meanBin(const vector<float> hist) {
    float sum = 0;
    float moment = 0;
    for (int i=0;i<hist.size();i++) {
        sum += hist[i];
        moment += i*hist[i];
    }
    return (int)(sum>0 ? moment/sum : 0);
}

int medianBin(const vector<float> hist) {
    float pos = getCount(hist) / 2;
    float sum = 0;
    int i = 0;
    for (;i<hist.size();i++) {
        sum += hist[i];
        if (pos <= sum) {
            break;
        }
    }
    // for (int k=0;k<hist.size();k++) {
      // cout << hist[k] << " ";
    // }
    // cout << pos*2 << " " << i << endl;
    return i;
}

// Get the highest bin
int modeBin(const vector<float> hist) {
    float max = 0;
    int maxIdx = 0;
    for (int i=0;i<hist.size();i++) {
        if (hist[i] > max) {
            max = hist[i];
            maxIdx = i;
        }
    }
    return maxIdx;
}

// Gets the vector of bins that are between target +- width inclusive
// Eg. target = 4, width = 1, returns bins 3,4,5
void getRange(const vector<float> hist, int target, int width, vector<float> &results) {
    results.resize(2*width+1);
    for (int i=0;i<2*width+1;i++) {
        if (target-width+i < 0 || target-width+i >= hist.size()) {
            results[i] = 0;
        } else {
            results[i] = hist[target-width+i];
        }
    }
    return;
}

// Normalize so that results are not dependent on density of point cloud in that region
void normalize(vector<float> &hist) {
  float sum = getCount(hist) + 40;
  if (sum > 0.5) {
      for (int i=0;i<hist.size();i++) {
          hist[i] = hist[i] / sum;
      }
  }
}

vector<float> calcNonlinearFeatures(vector<float> hist) {
    unsigned int b = hist.size() / 3;
    vector<float> f;
    f.reserve(b*3);
    for (int i=0;i<b;i++) {
        f.push_back(hist[i] * hist[2*b + i]);   // 1*3
        f.push_back(hist[i] / (hist[b + i]+1));     // 1/2
        f.push_back(hist[2*b + i] / (hist[b + i]+1));   // 3/2
    }
    return f;
}

// Get the length 21 feature vector for a given rect
void getHistogramFeatureVector(const IntegralHistogram *intHist, int nBins, int r, int c, int h, int w, vector<float> &results, bool useMean) {
    vector<float> hist;
    int mean;
    // if (useMean) mean = meanBin(hist);
    if (useMean) {
      getHistogramRect(intHist, r, c, h, w, hist);
		  mean = medianBin(hist);
    }
    else mean = (nBins)/2;
    
    results.clear();
    results.reserve(nBins * HISTOGRAM_ROWS * HISTOGRAM_COLS);
    for (int i=0;i<HISTOGRAM_ROWS;i++) {
        for (int j=0;j<HISTOGRAM_COLS;j++) {
            vector<float> t;
            getHistogramRect(intHist, 
                r + i*h/HISTOGRAM_ROWS,
                c + j*w/HISTOGRAM_COLS,
                h/HISTOGRAM_ROWS,
                w/HISTOGRAM_COLS,
                hist);
            normalize(hist);
            getRange(hist,mean,nBins/2,t);
            for (int k=0;k<nBins;k++) {
                results.push_back(t[k]);
            }
        }
    }
}

void getHistogramFeatureVectorDirect(const CvMat *mat, int nBins, float binWidth, const CvMat *mask, vector<float> &results, bool useMean) {
  if (mask != NULL && cvCountNonZero(mask) == 0) {
      results.resize(nBins*HISTOGRAM_ROWS * HISTOGRAM_COLS);
      for (int i=0;i<nBins*HISTOGRAM_ROWS * HISTOGRAM_COLS;i++)
          results[i] = 0;
      return;
  }
  double minVal, maxVal;
  cvMinMaxLoc(mat, &minVal, &maxVal, NULL, NULL, mask);
  minVal -= fmod(minVal,binWidth);
  maxVal -= fmod(maxVal,binWidth) - binWidth;
  if (!useMean || minVal < 0) minVal = 0.0;
  maxVal = (maxVal < minVal ? minVal + binWidth : maxVal);
  int nBinsTotal = (int)((maxVal-minVal)/binWidth);
	//~ printf("\n\nMIN: %f, MAX: %f, nBins: %d\n\n",minVal, maxVal, nBinsTotal);
  float ranges[nBinsTotal+1];
  ranges[0] = minVal; ranges[nBinsTotal] = maxVal;
  for (int i=1;i<nBinsTotal;i++) {
    ranges[i] = minVal + (maxVal - minVal)/((double)nBinsTotal) * ((double)i);
  }
  IntegralHistogram *intHist = calcIntegralHistogram(mat,nBinsTotal,ranges,mask);
  getHistogramFeatureVector(intHist,nBins,0,0,mat->height,mat->width,results,useMean);
  releaseIntegralHistogram(&intHist);
}
