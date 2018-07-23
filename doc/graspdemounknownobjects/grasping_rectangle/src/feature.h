#pragma once
#ifndef FEATURE_H
#define FEATURE_H

#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

#include "opencv2/core/core_c.h"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv/highgui.h"
#include "opencv2/video/tracking.hpp"
#include "opencv2/video/background_segm.hpp"

#include <stdio.h>
//#include "Utility.h"
#include <math.h>
#include <vector>
#include <iostream>
#include <fstream>
#include "ros/ros.h"
#include "common.h"

#include "pcl/point_types.h"
#include "pcl/kdtree/kdtree.h"
#include "pcl/features/normal_3d.h"
#include "pcl/features/fpfh.h"
#include "svm/svm_common.h"
#include "pcl/io/io.h"
#include "pcl/io/pcd_io.h"
//#include <queue>

#define USE_MOG_BG_MODEL 1

#define IMAGE_WIDTH 640
#define IMAGE_HEIGHT 480

#define MAGIC -1234.56
#define BORDER_SIZE 2
#define USE_SIZE false
#define USE_POINT_CLOUD true
#define USE_POINT_CLOUD_RATIOS true
#define USE_POINT_CLOUD_FPFH true
#define USE_IMAGE_FEATURE true
#define USE_IMAGE_FEATURE_RATIOS true

#define POINT_CLOUD_N_BINS 15
// #define POINT_CLOUD_FPFH_N_BINS 5
#define IMAGE_N_BINS 15

#define USE_BACKGROUND_SUBTRACTION true

#if USE_MOG_BG_MODEL == 1
CvGaussBGStatModelParams *getStatParams();	
#else
CvFGDStatModelParams *getStatParams();
#endif

using namespace std;

// Rotates the image without cropping
IplImage *imrotate(CvMat *mat, double angle, double magic=MAGIC);
IplImage *imrotate(IplImage * img, double angle, double magic=MAGIC);
IplImage *imresize(IplImage * img, double xScale, double yScale);
IplImage *imcrop(IplImage *img);
DOC *vectorToDoc(vector<float> vec);
DOC *extractFeature(CvMat * mat, CvRect rect);
double eval(CvMat * mat, CvRect rect, MODEL * model);
double eval(DOC* doc, MODEL* model);
IplImage *loadAndConvertImage(const char *filename, int cropX);
MODEL *readAndInitModel(const char *filename);
CvMat *backgroundSubtraction(IplImage *fore, IplImage *back);

void init();
void initFeatures();

void loadPCDFile(const char* filename, pcl::PointCloud<pcl::PointXYZ> &cloud, vector<int> &indices);
void loadAnglesFile(const char* filename, vector<float> &joint_angles, vector<float> &t_matrix);
// WARNING: Overwrites cloud!
void transform_to_global_frame(pcl::PointCloud<pcl::PointXYZ> &cloud, const vector<float> t);

void calculateNormals(pcl::PointCloud<pcl::PointXYZ> &cloud, pcl::PointCloud<pcl::Normal> &normals,
        int normals_k_neighbors);

void calculateFPFH(pcl::PointCloud<pcl::PointXYZ> &cloud, pcl::PointCloud<pcl::Normal> &normals,
        pcl::PointCloud<pcl::FPFHSignature33> &fpfh, int k_neighbors);
				
void extractRect(CvArr *mat, Rect r, IplImage* &result);

vector<CvMat*> convertNormalsToCvMat(const vector<int> indices, const pcl::PointCloud<pcl::Normal> normals, bool orientNormals = false);
vector<CvMat*> convertXYZToCvMat(const vector<int> indices, const pcl::PointCloud<pcl::PointXYZ> cloud);
vector<CvMat*> convertFPFHToCvMat(const vector<int> indices, const pcl::PointCloud<pcl::FPFHSignature33> fpfh);

// From SVMDataGenerator
void initFilters();
CvMat* getFilter(int idx);
void calculateFilterBanks17(IplImage* img, vector<CvMat*>& H);


#endif // FEATURE_H
