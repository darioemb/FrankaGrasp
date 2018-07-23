#pragma once
#ifndef EXTRACT_TRAINING_DATA_H
#define EXTRACT_TRAINING_DATA_H

#include "common.h"
#include "feature.h"
#include "histogram.h"

vector< vector<float> > extractTrainingData(string dir, int fileNum, vector<Rect> rects) {
	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::PointCloud<pcl::Normal> normals;
	pcl::PointCloud<pcl::FPFHSignature33> fpfh;
	vector<int> indices;
	int normals_k_neighbors = 50;
	int fpfh_k_neighbors = 10;
	double minVal, maxVal;
	vector<CvMat*> matXYZ, matNorms, matFPFH;
	vector<CvMat*> banks17Features;
	
	vector<float> hist;
	vector<float> feature_vec;
	vector< vector<float> > all_feature_vecs;
	vector<float> vecZ, vecNormZ, vecCurv;
	IplImage *m, *m2, *colorImg;
	CvMat *mm, *mmask; CvMat tmp, tmp2;
	float binWidth;
	int nBins = 9;
	
	char buff[20];
	sprintf(buff,"%04d",fileNum);

	loadPCDFile((dir+"pcd"+buff+".txt").c_str(),cloud,indices);
	calculateNormals(cloud,normals,normals_k_neighbors);
	calculateFPFH(cloud,normals,fpfh,fpfh_k_neighbors);
	matXYZ = convertXYZToCvMat(indices, cloud);
	matNorms = convertNormalsToCvMat(indices, normals, true);
	matFPFH = convertFPFHToCvMat(indices, fpfh);
	
	colorImg = cvLoadImage((dir+"pcd"+buff+"r.png").c_str());
	calculateFilterBanks17(colorImg, banks17Features);
	
	for (int rectNum = 0;rectNum < rects.size(); rectNum++) {
      // cout << rectNum << endl;
			extractRect(matXYZ[3], rects[rectNum],m2);
			mmask = cvGetMat(m2, &tmp2);
			// Need to erode the mask due to interpolation errors from rotating
			cvErode(mmask,mmask);
			feature_vec.clear();
			int fvsize = (USE_SIZE ? 2:0) 
				+ (USE_POINT_CLOUD ? POINT_CLOUD_N_BINS * 3 * HISTOGRAM_ROWS * HISTOGRAM_COLS : 0) 
				+ (USE_IMAGE_FEATURE ? 17 * HISTOGRAM_ROWS * HISTOGRAM_COLS * IMAGE_N_BINS : 0);
			// fvsize += (USE_POINT_CLOUD && USE_POINT_CLOUD_FPFH) ? 33 * HISTOGRAM_ROWS * HISTOGRAM_COLS * POINT_CLOUD_FPFH_N_BINS : 0;
			fvsize += (USE_POINT_CLOUD && USE_POINT_CLOUD_FPFH) ? 33 * HISTOGRAM_ROWS * HISTOGRAM_COLS : 0;
			fvsize += (USE_IMAGE_FEATURE && USE_IMAGE_FEATURE_RATIOS && HISTOGRAM_ROWS == 3) ? 17 * IMAGE_N_BINS * 3 * HISTOGRAM_COLS : 0;
			fvsize += (USE_POINT_CLOUD && USE_POINT_CLOUD_RATIOS && HISTOGRAM_ROWS == 3) ? 3 * POINT_CLOUD_N_BINS * 3 * HISTOGRAM_COLS : 0;
      feature_vec.reserve(fvsize);
			if (USE_SIZE) {
				feature_vec.push_back(mmask->width);
				feature_vec.push_back(mmask->height);
			}
			if (USE_POINT_CLOUD) {
				// For z values
				binWidth = 0.5 / POINT_CLOUD_N_BINS;
				extractRect(matXYZ[2], rects[rectNum],m);
				mm = cvGetMat(m, &tmp);
        
                
				// cvReleaseImage(&m2);
				// extractRect(matXYZ[3], rects[rectNum],m2);
				// mmask = cvGetMat(m2, &tmp2);
				// if (cvCountNonZero(mmask) == 0) {	// No point cloud info in this rect
					// for (int i=0;i<3*POINT_CLOUD_N_BINS*HISTOGRAM_ROWS * HISTOGRAM_COLS;i++)
						// feature_vec.push_back(0);
					// if (USE_POINT_CLOUD_FPFH) {
						// for (int i=0;i<33*POINT_CLOUD_FPFH_N_BINS*HISTOGRAM_ROWS * HISTOGRAM_COLS;i++) {
							// feature_vec.push_back(0);
						// }
					// }
				// } else {
					// printf("w: %d, h: %d, t: %d\n",mm->width,mm->height,mm->width*mm->height);
					// cvMinMaxLoc(mm, &minVal, &maxVal, NULL, NULL, mmask); // Z-axis
					// printf("min: %f, max: %f\n",minVal,maxVal);
					// Get the vector for 
          // cout << "before z...";
					getHistogramFeatureVectorDirect(mm,POINT_CLOUD_N_BINS,binWidth,mmask,vecZ,true);
          // cout << "after z"<< endl;
					cvReleaseImage(&m);
					// cvReleaseImage(&m2);
					
					// For normZ values
					binWidth = 1.0 / POINT_CLOUD_N_BINS;
					extractRect(matNorms[2], rects[rectNum],m);
					mm = cvGetMat(m, &tmp);
					// extractRect(matNorms[4], rects[rectNum],m2);
					// mmask = cvGetMat(m2, &tmp2);
					// Need to erode the mask due to interpolation errors from rotating
					// cvErode(mmask,mmask);
					// printf("w: %d, h: %d, t: %d\n",mm->width,mm->height,mm->width*mm->height);
					// cvMinMaxLoc(mm, &minVal, &maxVal, NULL, NULL, mmask); // Z-axis
					// printf("min: %f, max: %f\n",minVal,maxVal);
					// Get the vector for 
					getHistogramFeatureVectorDirect(mm,POINT_CLOUD_N_BINS,binWidth,mmask,vecNormZ,false);
					cvReleaseImage(&m);
					// cvReleaseImage(&m2);
					
					// For curv values
					binWidth = 0.1 / POINT_CLOUD_N_BINS;
					extractRect(matNorms[3], rects[rectNum],m);
					mm = cvGetMat(m, &tmp);
					// extractRect(matNorms[4], rects[rectNum],m2);
					// mmask = cvGetMat(m2, &tmp2);
					// Need to erode the mask due to interpolation errors from rotating
					// cvErode(mmask,mmask);
					// printf("w: %d, h: %d, t: %d\n",mm->width,mm->height,mm->width*mm->height);
					// cvMinMaxLoc(mm, &minVal, &maxVal, NULL, NULL, mmask); // Z-axis
					// printf("min: %f, max: %f\n",minVal,maxVal);
					// Get the vector for 
					getHistogramFeatureVectorDirect(mm,POINT_CLOUD_N_BINS,binWidth,mmask,vecCurv,false);
					cvReleaseImage(&m);
					// Put in the point cloud features
					feature_vec.insert(feature_vec.end(), vecZ.begin(), vecZ.end());
                    feature_vec.insert(feature_vec.end(), vecNormZ.begin(), vecNormZ.end());
					feature_vec.insert(feature_vec.end(), vecCurv.begin(), vecCurv.end());
					if (USE_POINT_CLOUD_RATIOS && HISTOGRAM_ROWS == 3) {
						vector<float> nl;
						nl = calcNonlinearFeatures(vecZ);
						feature_vec.insert(feature_vec.end(), nl.begin(), nl.end());
						nl = calcNonlinearFeatures(vecNormZ);
						feature_vec.insert(feature_vec.end(), nl.begin(), nl.end());
						nl = calcNonlinearFeatures(vecCurv);
						feature_vec.insert(feature_vec.end(), nl.begin(), nl.end());
					}
					if (USE_POINT_CLOUD_FPFH) {
						for (int k=0;k<matFPFH.size();k++) {
							vector<float> fv;
							extractRect(matFPFH[k], rects[rectNum], m);
							mm = cvGetMat(m, &tmp);
              // cvMinMaxLoc(mm, &minVal, &maxVal, NULL, NULL, mmask); // fpfh
              // printf("min: %f, max: %f\n",minVal, maxVal);
              cvThreshold(mm, mm, 0.0, 1, CV_THRESH_TOZERO);
              CvMat *sumMat = cvCreateMat(mm->height+1,mm->width+1,CV_64FC1);
              cvIntegral(mm, sumMat);
              int h = mm->height;
              int w = mm->width;
              int r = 0, c = 0;
              for (int i=0;i<HISTOGRAM_ROWS;i++) {
                for (int j=0;j<HISTOGRAM_COLS;j++) {
                  feature_vec.push_back(getCvIntegralRect(sumMat, 
                    r + i*h/HISTOGRAM_ROWS,
                    c + j*w/HISTOGRAM_COLS,
                    h/HISTOGRAM_ROWS,
                    w/HISTOGRAM_COLS));
                }
              }
              cvReleaseMat(&sumMat);
              
							// getHistogramFeatureVectorDirect(mm,POINT_CLOUD_FPFH_N_BINS,1.0/POINT_CLOUD_FPFH_N_BINS,mmask,fv,false);
							// feature_vec.insert(feature_vec.end(), fv.begin(), fv.end());
							cvReleaseImage(&m);
						}
					}
					cvReleaseImage(&m2);
				// }
			}
			if (USE_IMAGE_FEATURE) {
			    // vector<float> ratios;
			    // if (USE_IMAGE_FEATURE_RATIOS && HISTOGRAM_ROWS == 3) {
			        // ratios.reserve(banks17Features.size() * IMAGE_N_BINS * 3 * HISTOGRAM_COLS);
			    // }
				
			    // For Banks 17 features
				for (int k=0;k<banks17Features.size();k++) {
					vector<float> fv;
					extractRect(banks17Features[k], rects[rectNum], m);
					mm = cvGetMat(m, &tmp);
					mmask = cvCreateMat(mm->height, mm->width, CV_8UC1);
					
					cvThreshold(mm, mmask, 1.0, 1, CV_THRESH_BINARY_INV);
					getHistogramFeatureVectorDirect(mm,IMAGE_N_BINS,1.0/IMAGE_N_BINS,mmask,fv,false);
					// if (USE_IMAGE_FEATURE_RATIOS && HISTOGRAM_ROWS == 3) {
              // for (int u=0;u<HISTOGRAM_COLS;u++) {
                  // for (unsigned int q = 0; q<IMAGE_N_BINS;q++) {
                      // ratios.push_back(fv[u*IMAGE_N_BINS + q] * fv[(2*HISTOGRAM_COLS+u)*IMAGE_N_BINS + q]);
                      // ratios.push_back(fv[u*IMAGE_N_BINS + q] / fv[(1*HISTOGRAM_COLS+u)*IMAGE_N_BINS + q]);
                      // ratios.push_back(fv[(2*HISTOGRAM_COLS+u)*IMAGE_N_BINS + q] / fv[(1*HISTOGRAM_COLS+u)*IMAGE_N_BINS + q]);
                  // }
              // }
          // }
					feature_vec.insert(feature_vec.end(), fv.begin(), fv.end());
          if (USE_IMAGE_FEATURE_RATIOS && HISTOGRAM_ROWS == 3) {
            vector<float> nonlinearFeatures = calcNonlinearFeatures(fv);
            feature_vec.insert(feature_vec.end(), nonlinearFeatures.begin(), nonlinearFeatures.end());
          }
					cvReleaseImage(&m);
					cvReleaseMat(&mmask);
				}
				
				// For feature ratios
			    // if (USE_IMAGE_FEATURE_RATIOS && HISTOGRAM_ROWS == 3) {
			        // feature_vec.insert(feature_vec.end(), ratios.begin(), ratios.end());
			    // }
			}
			
			all_feature_vecs.push_back(feature_vec);
	}
	for (int i=0;i<matXYZ.size();i++) {
			cvReleaseMat(&(matXYZ[i]));
	}
	for (int i=0;i<matNorms.size();i++) {
			cvReleaseMat(&(matNorms[i]));
	}
	for (int i=0;i<banks17Features.size();i++) {
			cvReleaseMat(&(banks17Features[i]));
	}
	for (int i=0;i<matFPFH.size();i++) {
			cvReleaseMat(&(matFPFH[i]));
	}
	cvReleaseImage(&colorImg);
	return all_feature_vecs;
}

#endif // EXTRACT_TRAINING_DATA_H
