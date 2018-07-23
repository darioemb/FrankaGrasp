#include "common.h"
#include "feature.h"
#include "thr_pool.h"
#include "histogram.h"
#include <fstream>

using namespace std;

const int nTop = 1000;
vector < double >xSizes;
vector < double >ySizes;
vector < double >angles;
const double dx = 15, dy = 15;

CvRect CROP_RECT;


vector < ScoredRect > vRect;
pthread_mutex_t mutex;
static int work_count = 0;
static string BG_SAVE_FILENAME = "";

void init() {
	xSizes.clear();
	for (int i = 20; i <= 100; i += 15) {
		xSizes.push_back(i);
	}
	ySizes.clear();
	for (int i = 20; i <= 100; i += 15) {
		ySizes.push_back(i);
	}

	angles.clear();
	for (int i = 0; i < 12; i++)
		angles.push_back(((double)i) * 15);
	
}

//~ vector < ScoredRect > test(int img_num, MODEL *model, string bgFile);


typedef struct {
	IntegralHistogram *zIntHist, *normZIntHist, *curvIntHist;
	vector<IntegralHistogram*> banksHist;
  vector<CvMat*> fpfhHist;
	CvRect rect;
	ScoredRect r;
	MODEL *model;
} arg_t;
arg_t *arg(IntegralHistogram *zIntHist, IntegralHistogram *normZIntHist, IntegralHistogram *curvIntHist
        , vector<IntegralHistogram*> banksHist, vector<CvMat*> fpfhHist, CvRect rect, ScoredRect r, MODEL * model);

void *work(void *in);

typedef struct {
	CvMat *mat, **zMat, *tmp;
	IplImage **zImg;
	double angle;
} arg_RotateImg_t;
arg_RotateImg_t *arg_RotateImg(CvMat *mat, double angle, IplImage **zImg,  CvMat ** zMat, CvMat *tmp) {
	arg_RotateImg_t *t = new arg_RotateImg_t();
	t->mat = mat;
	t->angle = angle;
	t->zImg = zImg;
	t->zMat = zMat;
	t->tmp = tmp;
	return t;
}
void *work_RotateImg(void *in) {
	arg_RotateImg_t *t = (arg_RotateImg_t*) in;
	CvMat subMat;
	cvGetSubRect(t->mat, &subMat, CROP_RECT);
	*(t->zImg) = imrotate(&subMat, t->angle);
	*(t->zMat) = cvGetMat(*(t->zImg), t->tmp);
  delete t;
	return NULL;
}
typedef struct {
	IntegralHistogram **IntHist;
	CvMat *mat, *mask;
	int nBinsTotal;
	float *ranges;
} arg_CalcHist_t;
arg_CalcHist_t *arg_CalcHist(IntegralHistogram **IntHist, CvMat *mat, CvMat *mask, int nBinsTotal, float *ranges) {
	arg_CalcHist_t *t = new arg_CalcHist_t();
	t->IntHist = IntHist;
	t->mat = mat;
	t->mask = mask;
	t->nBinsTotal = nBinsTotal;
	t->ranges = ranges;
	return t;
}
void *work_CalcHist(void *in) {
	arg_CalcHist_t *t = (arg_CalcHist_t*) in;
	*(t->IntHist) = calcIntegralHistogram(t->mat, t->nBinsTotal, t->ranges, t->mask);
  delete t;
	return NULL;
}

typedef struct {
  CvMat* rotated_mat;
  CvMat** hist;
} arg_calc_fpfh_integral_hist_t;
arg_calc_fpfh_integral_hist_t *arg_calc_fpfh_integral_hist(
    CvMat** hist,
    CvMat* rotated_mat
  ) {
  arg_calc_fpfh_integral_hist_t  *t = new arg_calc_fpfh_integral_hist_t();
  t->rotated_mat = rotated_mat;
  t->hist = hist;
  return t;
}
void *work_calc_fpfh_integral_hist(void *in) {
	arg_calc_fpfh_integral_hist_t *t = (arg_calc_fpfh_integral_hist_t*) in;
  *(t->hist) = cvCreateMat(t->rotated_mat->height+1,t->rotated_mat->width+1,CV_64FC1);
  cvThreshold(t->rotated_mat, t->rotated_mat, 0.0, 1, CV_THRESH_TOZERO);
  cvIntegral(t->rotated_mat, *(t->hist));
  delete t;
	return NULL;
}

vector < ScoredRect > test(const char *imgFilename, const char *bgFilename, const char *pcdFilename, MODEL *model);

vector < ScoredRect > test(string dirPrefix, int img_num, MODEL *model, string bgFile) {
	
	char sarr[50];
	sprintf(sarr,"%04d",img_num);
	string filenum = sarr;
	string imgFile = (dirPrefix+"pcd"+filenum+"r.png").c_str();
	string pcdFile = (dirPrefix+"pcd"+filenum+".txt").c_str();
	return test(imgFile.c_str(), bgFile.c_str(), pcdFile.c_str(), model);
}

vector < ScoredRect > test(const char *imgFilename, const char *bgFilename, const char *pcdFilename, MODEL *model) {

	string filenum;
	const char *chrptr = strrchr(imgFilename,'/');
	int start = (chrptr != NULL) ? (int)(chrptr - imgFilename) : 0;
	while (start < strlen(imgFilename) && imgFilename[start] < 48 || imgFilename[start] > 57)
		start ++;
	if (start < strlen(imgFilename) - 4) {
		char tmp[5];
		for (int i=0;i<4;i++)
			tmp[i] = imgFilename[start+i];
		tmp[4] = '\0';
		filenum = string(tmp);
	} else {
		filenum = string("0000");
	}
	
	string imgFile = imgFilename;
	string pcdFile = pcdFilename;
	string bgFile = bgFilename;
	
	vRect.clear();
	vRect.resize(0);
	work_count = 0;
	
	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::PointCloud<pcl::Normal> normals;
  pcl::PointCloud<pcl::FPFHSignature33> fpfh;
	vector<int> indices;
	int normals_k_neighbors = 50;
  int fpfh_k_neighbors = 10;
	double minVal, maxVal;
	vector<CvMat*> matXYZ, matNorms, matFPFH;
	IplImage *zImg, *normZImg, *curvImg, *maskImg, *colorImg, *bgImg;
	vector<IplImage*> banks17FeaturesImageRotated;
	vector<IplImage*> fpfh33FeaturesImageRotated;
	CvMat *zMat, *mask, *bgMask, *bgMat;
	CvMat *normZMat;
	CvMat *curvMat;
	vector<CvMat*> banks17Features, banks17FeaturesRotated;
	vector<CvMat*> fpfh33Features, fpfh33FeaturesRotated;
	CvMat tmp1, tmp2, tmp3, tmp4, tmp5; // Used to hold cvMats
	CvMat tmp17[17];
	CvMat tmp33[33];
	vector<float> hist;
	vector<float> feature_vec;
	vector<float> vecZ, vecNormZ, vecCurv;
	float binWidth;

	
	if (USE_IMAGE_FEATURE || USE_BACKGROUND_SUBTRACTION) {
		colorImg = cvLoadImage(imgFile.c_str());
		//cvShowImage("bg",colorImg);
		//cvWaitKey(0);
		if (USE_IMAGE_FEATURE) {
			calculateFilterBanks17(colorImg, banks17Features);
		}
		if (USE_BACKGROUND_SUBTRACTION) {
			IplImage* background = cvLoadImage(bgFile.c_str());
			//cvShowImage("bg",background);
			//cvWaitKey(0);
			bgMat = backgroundSubtraction(colorImg, background);
            if (BG_SAVE_FILENAME.length() > 0) {
                IplImage bg;
                cvGetImage(bgMat, &bg);
                cvSaveImage(BG_SAVE_FILENAME.c_str(), &bg);
            }
			cvReleaseImage(&background);
		}
	}
	printf("load pcd...");
	loadPCDFile(pcdFile.c_str(), cloud, indices);
	printf("done\nConvertXYZtoCvMat...");
	matXYZ = convertXYZToCvMat(indices, cloud);
	printf("done\nCalculateNormals...");
	calculateNormals(cloud, normals, normals_k_neighbors);
	printf("done\nconvertNormalstoCvMat...");
	matNorms = convertNormalsToCvMat(indices, normals, true);
	printf("done\n");
	if (USE_POINT_CLOUD) {
		if (USE_POINT_CLOUD_FPFH) {
			printf("calcFPFH...");
			calculateFPFH(cloud,normals,fpfh,fpfh_k_neighbors);
			matFPFH = convertFPFHToCvMat(indices, fpfh);
			printf("done\n");
		}
	}
	cloud.points.clear();
	normals.points.clear();
	fpfh.points.clear();
	indices.clear();
    
	float totalWidth = CROP_RECT.width;
	float totalHeight = CROP_RECT.height;

	thr_pool_t *pool;
	int nWork = 0;
	long t1 = get_runtime();
	CvMat *pts = cvCreateMat(3, 4, CV_32FC1);
	for (int k = 0; k < 4; k++)
		cvmSet(pts, 2, k, 1.0);
	CvMat *rotatedPts = cvCreateMat(2, 4, CV_32FC1);
	CvMat *rotMatrix = cvCreateMat(2, 3, CV_32FC1);
	for (unsigned int c = 0; c < angles.size(); c++) {
		float *ranges, *ranges_fpfh, *rangesZ, *rangesNormZ, *rangesCurv;
		int nBinsTotal;
		IntegralHistogram *zIntHist, *normZIntHist, *curvIntHist;
		vector<IntegralHistogram*> banksHist;
		vector<CvMat*> fpfhHist;
		
		pool = thr_pool_create(4, 4, 0, NULL);
		
		thr_pool_queue(pool, work_RotateImg, arg_RotateImg(matXYZ[3], angles[c], &maskImg,  &mask, &tmp4));
		thr_pool_queue(pool, work_RotateImg, arg_RotateImg(matNorms[2], angles[c], &normZImg,  &normZMat, &tmp2));
				
		if (USE_POINT_CLOUD) {
			thr_pool_queue(pool, work_RotateImg, arg_RotateImg(matXYZ[2], angles[c], &zImg,  &zMat, &tmp1));
			thr_pool_queue(pool, work_RotateImg, arg_RotateImg(matNorms[3], angles[c], &curvImg,  &curvMat, &tmp3));
			if (USE_POINT_CLOUD_FPFH) {
				fpfhHist.resize(matFPFH.size());
				fpfh33FeaturesImageRotated.resize(matFPFH.size());
				fpfh33FeaturesRotated.resize(matFPFH.size());
				for (unsigned int i=0;i<matFPFH.size();i++) {
					thr_pool_queue(pool, work_RotateImg, arg_RotateImg(matFPFH[i], angles[c], &(fpfh33FeaturesImageRotated[i]),  &(fpfh33FeaturesRotated[i]), &(tmp33[i])));
				}
			}
		}
		
		if (USE_BACKGROUND_SUBTRACTION) {
			thr_pool_queue(pool, work_RotateImg, arg_RotateImg(bgMat, angles[c], &bgImg,  &bgMask, &tmp5));
		}
		if (USE_IMAGE_FEATURE) {
			banksHist.resize(banks17Features.size());
			banks17FeaturesImageRotated.resize(banks17Features.size());
			banks17FeaturesRotated.resize(banks17Features.size());
			for (unsigned int i=0;i<banks17Features.size();i++) {
				thr_pool_queue(pool, work_RotateImg, arg_RotateImg(banks17Features[i], angles[c], &(banks17FeaturesImageRotated[i]),  &(banks17FeaturesRotated[i]), &(tmp17[i])));
			}
		}
		
		thr_pool_wait(pool);
		thr_pool_destroy(pool);
		
		// Need to erode the mask due to interpolation errors from rotating
		cvErode(mask, mask);
		
		cv2DRotationMatrix(cvPoint2D32f
		        (mask->width / 2.0, mask->height / 2.0),
		        -angles[c], 1.0, rotMatrix);

		pool = thr_pool_create(4, 4, 0, NULL);

		if (USE_IMAGE_FEATURE) {
			// Calculate integral histogram for banks 17 features
			binWidth = 1.0 / IMAGE_N_BINS;
			minVal = 0; maxVal = 1;
			nBinsTotal = IMAGE_N_BINS;
			ranges = new float[nBinsTotal+1];
			ranges[0] = minVal;
			ranges[nBinsTotal] = maxVal;
			for (int i = 1;i < nBinsTotal;i++) {
				ranges[i] = minVal + (maxVal - minVal) / ((double)nBinsTotal) * ((double)i);
			}
			for (unsigned int i=0;i<banks17Features.size();i++) {
				thr_pool_queue(pool, work_CalcHist, arg_CalcHist(&(banksHist[i]), banks17FeaturesRotated[i], NULL, nBinsTotal, ranges));
			}
		}
		
		if (USE_POINT_CLOUD) {
			binWidth = 0.5 / POINT_CLOUD_N_BINS;
			cvMinMaxLoc(zMat, &minVal, &maxVal, NULL, NULL, mask);
			minVal -= fmod(minVal, binWidth);
			maxVal -= fmod(maxVal, binWidth) - binWidth;
			minVal = (minVal < 0.0 ? 0.0 : minVal);
      maxVal = (maxVal < minVal ? minVal + binWidth : maxVal);
			nBinsTotal = (int)((maxVal - minVal) / binWidth);
			rangesZ = new float[nBinsTotal+1];
			rangesZ[0] = minVal;
			rangesZ[nBinsTotal] = maxVal;
			for (int i = 1;i < nBinsTotal;i++) {
				rangesZ[i] = minVal + (maxVal - minVal) / ((double)nBinsTotal) * ((double)i);
			}
			thr_pool_queue(pool, work_CalcHist, arg_CalcHist(&zIntHist, zMat, mask, nBinsTotal, rangesZ));
			// zIntHist = calcIntegralHistogram(zMat, nBinsTotal, rangesZ, mask);
			
			binWidth = 1.0 / POINT_CLOUD_N_BINS;
			cvMinMaxLoc(normZMat, &minVal, &maxVal, NULL, NULL, mask);
			minVal -= fmod(minVal, binWidth);
			maxVal -= fmod(maxVal, binWidth) - binWidth;
			minVal = (minVal < 0.0 ? 0.0 : minVal);
      maxVal = (maxVal < minVal ? minVal + binWidth : maxVal);
			nBinsTotal = (int)((maxVal - minVal) / binWidth);
			rangesNormZ = new float[nBinsTotal+1];
			rangesNormZ[0] = minVal;
			rangesNormZ[nBinsTotal] = maxVal;
			for (int i = 1;i < nBinsTotal;i++) {
				rangesNormZ[i] = minVal + (maxVal - minVal) / ((double)nBinsTotal) * ((double)i);
			}
			thr_pool_queue(pool, work_CalcHist, arg_CalcHist(&normZIntHist, normZMat, mask, nBinsTotal, rangesNormZ));
			// normZIntHist = calcIntegralHistogram(normZMat, nBinsTotal, rangesNormZ, mask);
			
			binWidth = 0.1 / POINT_CLOUD_N_BINS;
			cvMinMaxLoc(curvMat, &minVal, &maxVal, NULL, NULL, mask);
			minVal -= fmod(minVal, binWidth);
			maxVal -= fmod(maxVal, binWidth) - binWidth;
			minVal = (minVal < 0.0 ? 0.0 : minVal);
      maxVal = (maxVal < minVal ? minVal + binWidth : maxVal);
			nBinsTotal = (int)((maxVal - minVal) / binWidth);
			rangesCurv = new float[nBinsTotal+1];
			rangesCurv[0] = minVal;
			rangesCurv[nBinsTotal] = maxVal;
			for (int i = 1;i < nBinsTotal;i++) {
				rangesCurv[i] = minVal + (maxVal - minVal) / ((double)nBinsTotal) * ((double)i);
			}
			thr_pool_queue(pool, work_CalcHist, arg_CalcHist(&curvIntHist, curvMat, mask, nBinsTotal, rangesCurv));
			// curvIntHist = calcIntegralHistogram(curvMat, nBinsTotal, rangesCurv, mask);
			
			if (USE_POINT_CLOUD_FPFH) {
				// Calculate integral histogram for fpfh features
				for (unsigned int i=0;i<matFPFH.size();i++) {
          thr_pool_queue(pool, work_calc_fpfh_integral_hist, arg_calc_fpfh_integral_hist(&(fpfhHist[i]), fpfh33FeaturesRotated[i]));
					// fpfhHist[i] = cvCreateMat(fpfh33FeaturesRotated[i]->height+1,fpfh33FeaturesRotated[i]->width+1,CV_64FC1);
          // cvThreshold(fpfh33FeaturesRotated[i], fpfh33FeaturesRotated[i], 0.0, 1, CV_THRESH_TOZERO);
          // cvIntegral(fpfh33FeaturesRotated[i], fpfhHist[i]);
				}
        
				// binWidth = 101.0 / POINT_CLOUD_FPFH_N_BINS;
				// minVal = 0; maxVal = 101;
				// nBinsTotal = POINT_CLOUD_FPFH_N_BINS;
				// ranges_fpfh = new float[nBinsTotal+1];
				// ranges_fpfh[0] = minVal;
				// ranges_fpfh[nBinsTotal] = maxVal;
				// for (int i = 1;i < nBinsTotal;i++) {
					// ranges_fpfh[i] = minVal + (maxVal - minVal) / ((double)nBinsTotal) * ((double)i);
				// }		
				// for (unsigned int i=0;i<matFPFH.size();i++) {
					// thr_pool_queue(pool, work_CalcHist, arg_CalcHist(&(fpfhHist[i]), fpfh33FeaturesRotated[i], mask, nBinsTotal, ranges_fpfh));
				// }
			}
		}
		//CvMat* banksMask = cvCreateMat(banks17FeaturesRotated[0]->height, banks17FeaturesRotated[0]->width, CV_8UC1);
		// for (int i=0;i<banks17Features.size();i++) {
			//cvThreshold(banks17FeaturesImageRotated[i], banksMask, 1.0, 1, CV_THRESH_BINARY_INV);
			//banksHist[i] = calcIntegralHistogram(banks17FeaturesRotated[i], nBinsTotal, ranges, banksMask);
			// banksHist[i] = calcIntegralHistogram(banks17FeaturesRotated[i], nBinsTotal, ranges, NULL);
			//~ cout << "BANKS int Hist "<< i <<" calculated\n";
		// }
		thr_pool_wait(pool);
		thr_pool_destroy(pool);
		if (USE_IMAGE_FEATURE) {
			delete[] ranges;
			for (unsigned int i=0;i<banks17FeaturesImageRotated.size();i++) {
                // cvReleaseMat(&(banks17FeaturesRotated[i]));
				cvReleaseImage(&(banks17FeaturesImageRotated[i]));
			}
		}
		
		if (USE_POINT_CLOUD) {
			delete[] rangesZ;
			delete[] rangesNormZ;
			delete[] rangesCurv;
			cvReleaseImage(&zImg);
			cvReleaseImage(&curvImg);
			
			if (USE_POINT_CLOUD_FPFH) {
				// delete[] ranges_fpfh;
				for (unsigned int i=0;i<fpfh33FeaturesImageRotated.size();i++) {
                    // cvReleaseMat(&(fpfh33FeaturesRotated[i]));
					cvReleaseImage(&(fpfh33FeaturesImageRotated[i]));
				}
			}
		}

		for (unsigned int a = 0; a < xSizes.size(); a++) {
			for (unsigned int b = 0; b < ySizes.size(); b++) {
				int _dx = xSizes[a];
				int _dy = ySizes[b];
                if (_dy > _dx) // Don't want rectangles that are too narrow
                    continue;
				pool = thr_pool_create(8, 8, 0, NULL);
				printf("fnum: %s im_w:%d im_h:%d angle:%.1f box_w:%.0f box_h:%.0f\n", filenum.c_str(), mask->width, mask->height, angles[c], xSizes[a], ySizes[b]);
				for (int j = 0; j < mask->height - _dy; j += dy) { // Rows
					for (int i = 0; i < mask->width - _dx; i += dx) { // Cols
						//~ for (int j = 310; j < mask->height - _dy; j += dy) { // Rows
						//~ for (int i = 230; i < mask->width - _dx; i += dx) { // Cols
						// printf("normZMat: %p\n",normZMat);
						if ((cvmGet(normZMat, j, i) ) < 0 ||
						        (cvmGet(normZMat, j + _dy - 1, i) ) < 0 ||
						        (cvmGet(normZMat, j, i + _dx - 1) ) < 0 ||
						        (cvmGet(normZMat, j + _dy - 1, i + _dx - 1) ) < 0 ) {
							continue;
						}
						if ( USE_BACKGROUND_SUBTRACTION &&
								(cvGet2D(bgMask, j, i).val[0] <= 0 || 
								cvGet2D(bgMask, j + _dy - 1, i).val[0] <= 0 || 
								cvGet2D(bgMask, j, i + _dx - 1).val[0] <= 0 || 
								cvGet2D(bgMask, j + _dy - 1, i + _dx - 1).val[0] <= 0)
							) {
							continue;
						}
						cvmSet(pts, 0, 0, i);
						cvmSet(pts, 1, 0, j);
						cvmSet(pts, 0, 1, i);
						cvmSet(pts, 1, 1, (j + _dy));
						cvmSet(pts, 0, 2, (i + _dx));
						cvmSet(pts, 1, 2, (j + _dy));
						cvmSet(pts, 0, 3, (i + _dx));
						cvmSet(pts, 1, 3, j);

						cvGEMM(rotMatrix, pts, 1.0, NULL, 0.0, rotatedPts);
						vector<double> x, y;
						x.resize(4);
						y.resize(4);
						bool outlier = false;
						//~ cout << "ITER: " << i << ", " << j << endl;
						for (int k = 0;k < 4;k++) {
							//printf("points %d: %g %g\n",k,cvmGet(rotatedPts,0,k));
							x[k] = cvmGet(rotatedPts, 0, k) - (((float)(mask->width)) / 2.0) + (totalWidth / 2.0);
							y[k] = cvmGet(rotatedPts, 1, k) - (((float)(mask->height)) / 2.0) + (totalHeight / 2.0);
							//~ cout << x[k] << "," << y[k] << " ";
							if (x[k] < 0 || y[k] < 0 || x[k] >= totalWidth || y[k] >= totalHeight) {
								outlier = true;
								break;
							}
							//y[k] = cvmGet(rotatedPts,1,k)/yScales[b];
						}
						//~ cout << endl;
						if (outlier) continue;

						/*
						if (outlier) {
						    for (int p=0;p<4;p++)
						        printf("(%g, %g) ",x[p],y[p]);
						    printf("\n");
						    printf("%g %g %g %d %d\n",xScales[a],yScales[b],angles[c],i,j);
						    printf("%g %g %g %g\n",cvmGet(mat, j, i), cvmGet(mat, j + 19, i)
						        , cvmGet(mat, j + 19, i + 19), cvmGet(mat, j, i + 19));
						    printf("im_resized : %d %d\n", im_rotated->width, im_rotated->height);
						    printf("im_rotated : %d %d\n", im_resized->width, im_resized->height);
						    return -1;
						}*/
						ScoredRect sr = ScoredRect(Rect(x, y), 0.0);
						thr_pool_queue(pool, work, arg(zIntHist, normZIntHist, curvIntHist, banksHist, fpfhHist, cvRect(i, j, _dx, _dy), sr, model));
						// cout << "Before work\n";
						// work((void*)(arg(zIntHist, normZIntHist, curvIntHist, banksHist, fpfhHist, cvRect(i, j, _dx, _dy), sr, model)));
						// cout << "After work\n";
						nWork++;
					}
				}
				thr_pool_wait(pool);
				thr_pool_destroy(pool);
				sort(vRect.begin(), vRect.end());
				vRect.resize(min(nTop,(int) (vRect.size())));
				// cvReleaseMat(&mat);
				//~ cvReleaseImage(&im_resized);
				printf("%d %d %d: %d evals\n", a, b, c, nWork);
			}
		}
		//~ if (c != 0)
		//~ cvReleaseImage(&im_rotated);

		if (USE_IMAGE_FEATURE) {
			for (unsigned int i=0;i<banksHist.size();i++) {
				releaseIntegralHistogram(&(banksHist[i]));
			}
			// for (unsigned int i=0;i<banks17FeaturesImageRotated.size();i++) {
                // cvReleaseMat(&(banks17FeaturesRotated[i]));
				// cvReleaseImage(&(banks17FeaturesImageRotated[i]));
			// }
		}
		
		if (USE_POINT_CLOUD) {
			releaseIntegralHistogram(&zIntHist);
			releaseIntegralHistogram(&normZIntHist);
			releaseIntegralHistogram(&curvIntHist);
			
            // cvReleaseMat(&zMat);
            // cvReleaseMat(&curvMat);
			// cvReleaseImage(&zImg);
			// cvReleaseImage(&curvImg);
			
			if (USE_POINT_CLOUD_FPFH) {
				for (unsigned int i=0;i<fpfhHist.size();i++) {
					// releaseIntegralHistogram(&(fpfhHist[i]));
					cvReleaseMat(&(fpfhHist[i]));
				}
				// for (unsigned int i=0;i<fpfh33FeaturesImageRotated.size();i++) {
                    // cvReleaseMat(&(fpfh33FeaturesRotated[i]));
					// cvReleaseImage(&(fpfh33FeaturesImageRotated[i]));
				// }
			}
		}
		
		if (USE_BACKGROUND_SUBTRACTION) {
            // cvReleaseMat(&bgMask);
			cvReleaseImage(&bgImg);
		}
		
        // cvReleaseMat(&normZMat);
        // cvReleaseMat(&mask);
		cvReleaseImage(&normZImg);
		cvReleaseImage(&maskImg);
	}
	if (USE_IMAGE_FEATURE || USE_BACKGROUND_SUBTRACTION) {
		cvReleaseImage(&colorImg);
		if (USE_BACKGROUND_SUBTRACTION) {
			cvReleaseMat(&bgMat);
		}
		if (USE_IMAGE_FEATURE) {
			for (unsigned int i = 0;i < banks17Features.size();i++) {
				cvReleaseMat(&(banks17Features[i]));
			}
		}
	}
	
	if (USE_POINT_CLOUD) {
		if (USE_POINT_CLOUD_FPFH) {
			for (unsigned int i = 0;i < matFPFH.size();i++) {
				cvReleaseMat(&(matFPFH[i]));
			}
		}
	}
	cvReleaseMat(&rotMatrix);
	cvReleaseMat(&pts);
	cvReleaseMat(&rotatedPts);
	for (unsigned int i = 0;i < matXYZ.size();i++) {
		cvReleaseMat(&(matXYZ[i]));
	}
	for (unsigned int i = 0;i < matNorms.size();i++) {
		cvReleaseMat(&(matNorms[i]));
	}

	// printf("%d of %d done\n",work_count,nWork);
	// printf("%d of %d done\n",work_count,nWork);
	long runtime = get_runtime() - t1;
	printf("Runtime in cpu-seconds: %.4f\n", (float)(runtime / 100.0));
	return vRect;
}

arg_t *arg(IntegralHistogram *zIntHist, IntegralHistogram *normZIntHist, IntegralHistogram *curvIntHist
        , vector<IntegralHistogram*> banksHist, vector<CvMat*> fpfhHist, CvRect rect, ScoredRect r, MODEL * model) {
	arg_t *t = new arg_t();
	if (t == NULL)
		printf("MALLOC failed at arg!\n");
	t->zIntHist = zIntHist;
	t->normZIntHist = normZIntHist;
	t->curvIntHist = curvIntHist;
	t->banksHist = banksHist;
	t->fpfhHist = fpfhHist;
	t->rect = rect;
	t->model = model;
	t->r = r;
	return t;
}

void *work(void *in) {
	arg_t *t = (arg_t *) in;
	ScoredRect rr = t->r;
	vector<float> fv1, fv2, fv3;
	//~ cout << "\nFV1:\n";
	vector<float> fv;
	int fvsize = (USE_SIZE ? 2:0) 
		+ (USE_POINT_CLOUD ? POINT_CLOUD_N_BINS * 3 * HISTOGRAM_ROWS * HISTOGRAM_COLS : 0) 
		+ (USE_IMAGE_FEATURE ? 17 * HISTOGRAM_ROWS * HISTOGRAM_COLS * IMAGE_N_BINS : 0);
	// fvsize += (USE_POINT_CLOUD && USE_POINT_CLOUD_FPFH) ? 33 * HISTOGRAM_ROWS * HISTOGRAM_COLS * POINT_CLOUD_FPFH_N_BINS : 0;
  fvsize += (USE_POINT_CLOUD && USE_POINT_CLOUD_FPFH) ? 33 * HISTOGRAM_ROWS * HISTOGRAM_COLS : 0;
	fvsize += (USE_IMAGE_FEATURE && USE_IMAGE_FEATURE_RATIOS && HISTOGRAM_ROWS == 3) ? 17 * IMAGE_N_BINS * 3 * HISTOGRAM_COLS : 0;
  fvsize += (USE_POINT_CLOUD && USE_POINT_CLOUD_RATIOS && HISTOGRAM_ROWS == 3) ? 3 * POINT_CLOUD_N_BINS * 3 * HISTOGRAM_COLS : 0;
	fv.reserve(fvsize);
	if (USE_SIZE) {
		fv.push_back(t->rect.width);
		fv.push_back(t->rect.height);
	}
	if (USE_POINT_CLOUD) {
		getHistogramFeatureVector(t->zIntHist, POINT_CLOUD_N_BINS, (t->rect).y, (t->rect).x, (t->rect).height, (t->rect).width, fv1, true);
		//~ for (int i=0;i<fv1.size();i++) {
		//~ cout << fv1[i] << " ";
		//~ if (i%9 == 8) cout << endl;
		//~ }
		//~ cout << "\nFV2:\n";
		getHistogramFeatureVector(t->normZIntHist, POINT_CLOUD_N_BINS, (t->rect).y, (t->rect).x, (t->rect).height, (t->rect).width, fv2, false);
		//~ for (int i=0;i<fv2.size();i++) {
		//~ cout << fv2[i] << " ";
		//~ if (i%9 == 8) cout << endl;
		//~ }
		getHistogramFeatureVector(t->curvIntHist, POINT_CLOUD_N_BINS, (t->rect).y, (t->rect).x, (t->rect).height, (t->rect).width, fv3, false);
		fv.insert(fv.end(), fv1.begin(), fv1.end());
		fv.insert(fv.end(), fv2.begin(), fv2.end());
		fv.insert(fv.end(), fv3.begin(), fv3.end());
    if (USE_POINT_CLOUD_RATIOS && HISTOGRAM_ROWS == 3) {
      vector<float> nl;
      nl = calcNonlinearFeatures(fv1);
      fv.insert(fv.end(), nl.begin(), nl.end());
      nl = calcNonlinearFeatures(fv2);
      fv.insert(fv.end(), nl.begin(), nl.end());
      nl = calcNonlinearFeatures(fv3);
      fv.insert(fv.end(), nl.begin(), nl.end());
    }
		if (USE_POINT_CLOUD_FPFH) {
			for (unsigned int i=0;i<(t->fpfhHist).size();i++) {
        int r = (t->rect).y;
        int c = (t->rect).x;
        int h = (t->rect).height;
        int w = (t->rect).width;
        for (int i=0;i<HISTOGRAM_ROWS;i++) {
          for (int j=0;j<HISTOGRAM_COLS;j++) {
            fv.push_back(getCvIntegralRect(t->fpfhHist[i], 
              r + i*h/HISTOGRAM_ROWS,
              c + j*w/HISTOGRAM_COLS,
              h/HISTOGRAM_ROWS,
              w/HISTOGRAM_COLS));
          }
        }
				// vector<float> fpfh_fv;
                // if (t->fpfhHist[i] == NULL)
                    // cout << "ERROR!! FPFH NULL!\n";
				// cout << i << " getHistogramFeatureVector\n";
				// getHistogramFeatureVector(t->fpfhHist[i], POINT_CLOUD_FPFH_N_BINS, (t->rect).y, (t->rect).x, (t->rect).height, (t->rect).width, fpfh_fv, false);
				// fv.insert(fv.end(), fpfh_fv.begin(), fpfh_fv.end());
			}
			// for (int k=0;k<matFPFH.size();k++) {
				// vector<float> fv;
				// extractRect(matFPFH[k], rects[rectNum], m);
				// mm = cvGetMat(m, &tmp);
				// getHistogramFeatureVectorDirect(mm,POINT_CLOUD_FPFH_N_BINS,101.0/POINT_CLOUD_FPFH_N_BINS,mmask,fv,false);
				// feature_vec.insert(feature_vec.end(), fv.begin(), fv.end());
				// cvReleaseImage(&m);
			// }
		}
	}
	if (USE_IMAGE_FEATURE) {
	    // vector<float> ratios;
	    // if (USE_IMAGE_FEATURE_RATIOS && HISTOGRAM_ROWS == 3) {
	        // ratios.reserve((t->banksHist).size() * IMAGE_N_BINS * 3 * HISTOGRAM_COLS);
	    // }
		for (unsigned int i=0;i<(t->banksHist).size();i++) {
			vector<float> banks_fv;
			getHistogramFeatureVector(t->banksHist[i], IMAGE_N_BINS, (t->rect).y, (t->rect).x, (t->rect).height, (t->rect).width, banks_fv, false);
		    fv.insert(fv.end(), banks_fv.begin(), banks_fv.end());
        if (USE_IMAGE_FEATURE_RATIOS && HISTOGRAM_ROWS == 3) {
          // for (int u=0;u<HISTOGRAM_COLS;u++) {
            // for (unsigned int q = 0; q<IMAGE_N_BINS;q++) {
              // ratios.push_back(banks_fv[u*IMAGE_N_BINS + q] * banks_fv[(2*HISTOGRAM_COLS+u)*IMAGE_N_BINS + q]);
              // ratios.push_back(banks_fv[u*IMAGE_N_BINS + q] / banks_fv[(1*HISTOGRAM_COLS+u)*IMAGE_N_BINS + q]);
              // ratios.push_back(banks_fv[(2*HISTOGRAM_COLS+u)*IMAGE_N_BINS + q] / banks_fv[(1*HISTOGRAM_COLS+u)*IMAGE_N_BINS + q]);
            // }
          // }
          vector<float> nonlinearFeatures = calcNonlinearFeatures(banks_fv);
          fv.insert(fv.end(), nonlinearFeatures.begin(), nonlinearFeatures.end());
        }
		}
    // if (USE_IMAGE_FEATURE_RATIOS && HISTOGRAM_ROWS == 3) {
        // fv.insert(fv.end(), ratios.begin(), ratios.end());
    // }
	}
	//~ cout << "\nFV3:\n";
	// cout << fv.size() << endl;
	// for (int i=0;i<fv.size();i++) {
		// cout << fv[i] << " ";
		// if (i%9 == 8) cout << endl;
	// }
	//~ cout << fv.size() << endl;
	DOC *doc = vectorToDoc(fv);

	rr.score = eval(doc, t->model);
	// printf("%g\n",res);
	(void)pthread_mutex_lock(&mutex);
	vRect.push_back(rr);
	work_count++;
	(void)pthread_mutex_unlock(&mutex);
	delete t;
	return NULL;
}
