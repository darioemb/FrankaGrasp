#include "common.h"
#include "feature.h"
#include "thr_pool.h"
#include "testUtil.h"

// using namespace std;
//char* modelfile = "model.mod";
char *modelfile = "model_poly.mod";
//char *modelfile = "model_rbf.mod";
// char* imgFile = "opencvtut1-1.png";
//char *imgFile = "left0121.pgm";
//char *pcdFile = "pcd0121.txt";
//char *anglesFile = "pcd0121_angles.txt";
char *imgFile = "left0000.pgm";
char *pcdFile = "pcd0000.txt";
char *anglesFile = "pcd0000_angles.txt";
const int xCrop = 400;
const int nTop = 100;
vector < double >xScales;
vector < double >yScales;
vector < double >angles;
const double dx = 20, dy = 20;

int main(int argc, char *argv[]) {
	vector<float> joint_angles;
	vector<float> t_matrix;
	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::PointCloud<pcl::Normal> normals;
	vector<int> indices;
	int normals_k_neighbors = 50;
	double minVal, maxVal;
	vector<CvMat*> mat;

	loadAnglesFile(anglesFile, joint_angles, t_matrix);
	printf("Joint angles:\n");
	for (int i = 0;i < joint_angles.size();i++)
		printf("%f ", joint_angles[i]);
	printf("\nTransformation Matrix:\n");
	for (int i = 0;i < t_matrix.size();i++)
		printf("%f ", t_matrix[i]);
	printf("\n");

	printf("Camera frame:\n");
	loadPCDFile(pcdFile, cloud, indices);
	//calculateNormals(cloud,normals,normals_k_neighbors);
	/*for (int i=0;i<10;i++) {
	  printf("%d %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n",
	      indices[i],cloud.points[i].x,cloud.points[i].y,cloud.points[i].z,
	      normals.points[i].normal[0],normals.points[i].normal[1],normals.points[i].normal[2],
	      normals.points[i].curvature);
	}*/
	mat = convertXYZToCvMat(indices, cloud);
	cvMinMaxLoc(mat[2], &minVal, &maxVal, NULL, NULL, mat[3]); // Z-axis

	float binWidth = 0.05;
	minVal -= fmod(minVal, binWidth);
	maxVal -= fmod(maxVal, binWidth) - binWidth;
	int nBins = (int)((maxVal - minVal) / binWidth);
	float ranges[nBins+1];
	ranges[0] = minVal;
	ranges[nBins] = maxVal;
	for (int i = 1;i < nBins;i++) {
		ranges[i] = minVal + (maxVal - minVal) / ((double)nBins) * ((double)i);
	}
	IntegralHistogram *intHist = calcIntegralHistogram(mat[2], nBins, ranges, mat[3]);
	vector<float> hist;
	vector<float> rng;
	/*getHistogramRect(intHist, 0, 0, 768, 1024, hist);
	for (int i=0;i<nBins;i++) {
	  printf("%2d: %g to %g, %.0f\n",i,ranges[i],ranges[i+1],hist[i]);
	}
	getHistogramRect(intHist, 354, 462, 100, 100, hist);
	for (int i=0;i<nBins;i++) {
	  printf("%.0f ",hist[i]);
	}
	printf("\n");
	getRange(hist,meanBin(hist),3,rng);
	for (int i=0;i<rng.size();i++)
	  printf("%.0f ",rng[i]);
	printf("\n");
	getRange(hist,modeBin(hist),3,rng);
	for (int i=0;i<rng.size();i++)
	  printf("%.0f ",rng[i]);
	printf("\n");
	getHistogramRect(intHist, 254, 362, 100, 100, hist);
	for (int i=0;i<nBins;i++) {
	  printf("%.0f,",hist[i]);
	}
	printf("\n");
	getRange(hist,meanBin(hist),3,rng);
	for (int i=0;i<rng.size();i++)
	  printf("%.0f ",rng[i]);
	printf("\n");
	getRange(hist,modeBin(hist),3,rng);*/
	getHistogramFeatureVector(intHist, 354, 462, 100, 100, rng);
	for (int i = 0;i < rng.size();i++) {
		printf("%.3f ", rng[i]);
		if (i % 7 == 6) printf("\n");
	}
	printf("\n");
	getHistogramFeatureVector(intHist, 254, 362, 100, 100, rng);
	for (int i = 0;i < rng.size();i++) {
		printf("%.3f ", rng[i]);
		if (i % 7 == 6) printf("\n");
	}
	printf("\n");
	/*for (int i=0;i<intHist->w*intHist->h;i++) {
	  if (intHist->img[i] != 0) {
	      printf("%d %d\n",i,intHist->img[i]);
	  }
	}*/
	releaseIntegralHistogram(&intHist);
	return 0;
	/*
	mat = convertNormalsToCvMat(indices, normals);
	//vector<CvMat*> mat = convertXYZToCvMat(indices, cloud);

	cvNamedWindow("Results", CV_WINDOW_AUTOSIZE);
	for (int i=0;i<mat.size();i++) {
	    cvMinMaxLoc(mat[i], &minVal, &maxVal);
	    printf("min: %g max: %g\n",minVal,maxVal);
	    cvConvertScale(mat[i],mat[i],1.0/(maxVal-minVal),-minVal/(maxVal-minVal));
	    cvShowImage("Results", mat[i]);
	    cvWaitKey(0);
	}
	mat = convertXYZToCvMat(indices, cloud);
	for (int i=0;i<mat.size();i++) {
	    cvMinMaxLoc(mat[i], &minVal, &maxVal);
	    printf("min: %g max: %g\n",minVal,maxVal);
	    cvConvertScale(mat[i],mat[i],1.0/(maxVal-minVal),-minVal/(maxVal-minVal));
	    cvShowImage("Results", mat[i]);
	    cvWaitKey(0);
	}

	printf("Global frame:\n");
	transform_to_global_frame(cloud,t_matrix);
	calculateNormals(cloud,normals,normals_k_neighbors);
	for (int i=0;i<10;i++) {
	  printf("%d %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n",
	      indices[i],cloud.points[i].x,cloud.points[i].y,cloud.points[i].z,
	      normals.points[i].normal[0],normals.points[i].normal[1],normals.points[i].normal[2],
	      normals.points[i].curvature);
	}
	mat = convertNormalsToCvMat(indices, normals);
	//vector<CvMat*> mat = convertXYZToCvMat(indices, cloud);
	cvNamedWindow("Results", CV_WINDOW_AUTOSIZE);
	for (int i=0;i<mat.size();i++) {
	    cvMinMaxLoc(mat[i], &minVal, &maxVal);
	    printf("min: %g max: %g\n",minVal,maxVal);
	    cvConvertScale(mat[i],mat[i],1.0/(maxVal-minVal),-minVal/(maxVal-minVal));
	    cvShowImage("Results", mat[i]);
	    cvWaitKey(0);
	}

	mat = convertXYZToCvMat(indices, cloud);
	for (int i=0;i<mat.size();i++) {
	    cvMinMaxLoc(mat[i], &minVal, &maxVal);
	    printf("min: %g max: %g\n",minVal,maxVal);
	    cvConvertScale(mat[i],mat[i],1.0/(maxVal-minVal),-minVal/(maxVal-minVal));
	    cvShowImage("Results", mat[i]);
	    cvWaitKey(0);
	}*/

	//cvReleaseImage(&img);
	cvDestroyWindow("Results");
	return 0;
	init();
	IplImage *img = loadAndConvertImage(imgFile, xCrop);
	MODEL *model = readAndInitModel(modelfile);
	CvMat temp;
	/*
	   for(int i=0;i<10;i++) { for (int j=0;j<10;j++) { printf("%f
	   ",cvmGet(mat,i,j)); } printf("\n"); } */

	// cvWaitKey(0);
	thr_pool_t *pool;
	int nWork = 0;
	long t1 = get_runtime();
	CvMat *pts = cvCreateMat(3, 4, CV_32FC1);
	for (int k = 0; k < 4; k++)
		cvmSet(pts, 2, k, 1.0);
	CvMat *rotatedPts = cvCreateMat(2, 4, CV_32FC1);
	CvMat *rotMatrix = cvCreateMat(2, 3, CV_32FC1);
	for (unsigned int c = 0; c < angles.size(); c++) {
		IplImage *im_rotated;
		if (c == 0)
			im_rotated = img;
		else
			im_rotated = imrotate(img, angles[c]);
		cv2DRotationMatrix(cvPoint2D32f
		        (im_rotated->width / 2.0, im_rotated->height / 2.0),
		        -angles[c], 1.0, rotMatrix);
		for (unsigned int a = 0; a < xScales.size(); a++) {
			for (unsigned int b = 0; b < yScales.size(); b++) {
				IplImage *im_resized;
				im_resized = imresize(im_rotated, xScales[a], yScales[b]);
				CvMat *mat = cvGetMat(im_resized, &temp);
				int _dx = ceil(dx * xScales[a]);
				int _dy = ceil(dy * yScales[b]);
				pool = thr_pool_create(4, 4, 0, NULL);
				printf("%d %d %d %d\n", mat->width, mat->height, _dx, _dy);
				for (int i = 0; i < mat->width - 20; i += _dx) {
					for (int j = 0; j < mat->height - 20; j += _dy) {
						if ((cvmGet(mat, j, i) ) < 0 ||
						        (cvmGet(mat, j + 19, i) ) < 0 ||
						        (cvmGet(mat, j, i + 19) ) < 0 ||
						        (cvmGet(mat, j + 19, i + 19) ) < 0 ) {
							continue;
						}
						cvmSet(pts, 0, 0, (double)i / xScales[a]);
						cvmSet(pts, 1, 0, (double)j / yScales[b]);
						cvmSet(pts, 0, 1, (double)i / xScales[a]);
						cvmSet(pts, 1, 1, (double)(j + 20) / yScales[b]);
						cvmSet(pts, 0, 2, (double)(i + 20) / xScales[a]);
						cvmSet(pts, 1, 2, (double)(j + 20) / yScales[b]);
						cvmSet(pts, 0, 3, (double)(i + 20) / xScales[a]);
						cvmSet(pts, 1, 3, (double)j / yScales[b]);

						cvGEMM(rotMatrix, pts, 1.0, NULL, 0.0, rotatedPts);
						vector<double> x, y;
						x.resize(4);
						y.resize(4);
						bool outlier = false;
						for (int k = 0;k < 4;k++) {
							//printf("points %d: %g %g\n",k,cvmGet(rotatedPts,0,k));
							x[k] = xCrop + (cvmGet(rotatedPts, 0, k) - (im_rotated->width / 2.0) + (img->width / 2.0));
							y[k] = (cvmGet(rotatedPts, 1, k) - (im_rotated->height / 2.0) + (img->height / 2.0));
							if (x[k] < xCrop || y[k] < 0 || x[k] >= (img->width + xCrop) || y[k] >= img->height) {
								outlier = true;
								break;
							}
							//y[k] = cvmGet(rotatedPts,1,k)/yScales[b];
						}
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
						thr_pool_queue(pool, work, arg(mat, cvRect(i, j, 20, 20),
						        sr, model));
						nWork++;
						// eval(mat, cvRect(i, j, 20, 20), model);
					}
				}
				thr_pool_wait(pool);
				thr_pool_destroy(pool);
				sort(vRect.begin(), vRect.end());
				vRect.resize(nTop);
				// cvReleaseMat(&mat);
				cvReleaseImage(&im_resized);
				printf("%d %d %d: %d evals\n", a, b, c, nWork);
			}
		}
		if (c != 0)
			cvReleaseImage(&im_rotated);
	}
	cvReleaseMat(&rotMatrix);
	cvReleaseMat(&pts);
	cvReleaseMat(&rotatedPts);


	// printf("%d of %d done\n",work_count,nWork);
	// printf("%d of %d done\n",work_count,nWork);
	long runtime = get_runtime() - t1;
	printf("Runtime in cpu-seconds: %.4f\n", (float)(runtime / 100.0));
	//sort(vRect.begin(), vRect.end());
	cvReleaseImage(&img);
	img = cvLoadImage(imgFile);
	for (int i = 0; i < 20; i++) {
		ScoredRect sr = vRect[i];
		//printf("%.4g %g %g %g %g %g\n", sr.score, sr.x, sr.y, sr.w, sr.h, sr.t);
		vector<double> x = sr.rect.x;
		vector<double> y = sr.rect.y;
		printf("%.4g", sr.score);
		for (unsigned int k = 0;k < x.size();k++)
			printf(" %g %g", x[k], y[k]);
		printf("\n");
		CvPoint  box[] = {x[0], y[0],  x[1], y[1],  x[2], y[2],  x[3], y[3]};
		CvPoint* boxArr[1] = {box};
		int      nCurvePts[1] = {4};
		int      nCurves = 1;
		int      isCurveClosed = 0;
		int      lineWidth = 1;
		cvPolyLine(img, boxArr, nCurvePts, nCurves, isCurveClosed, cvScalar(255, 0, 0), lineWidth);
	}

	/* printf("feature: %ld %ld %f
	   %ld\n",feature->docnum,feature->queryid,feature->costfactor,feature->slackid);
	   SVECTOR* vec = feature->fvec; printf("vec: %f
	   %f\n",vec->twonorm_sq,vec->factor); WORD* w = vec->words; int i=0; while
	   (w[i].wnum) { printf("Word: %ld %f\n",w[i].wnum,w[i].weight); i++; } */

	/*
	   IplImage* img2 = imrotate(img, 30.0); cvShowImage("Example1", img2);
	   cvWaitKey(0); cvReleaseImage( &img2 ); IplImage* img3 = imrotate(img,
	   120); cvShowImage("Example1", img3); cvWaitKey(0); cvReleaseImage( &img3
	   ); IplImage* img4 = imrotate(img, 210); cvShowImage("Example1", img4);
	   cvWaitKey(0); cvReleaseImage( &img4 ); IplImage* img5 = imrotate(img,
	   300); cvShowImage("Example1", img5); cvWaitKey(0); cvReleaseImage( &img5
	   ); */
	cvNamedWindow("Results", CV_WINDOW_AUTOSIZE);
	cvShowImage("Results", img);
	cvWaitKey(0);
	cvReleaseImage(&img);
	cvDestroyWindow("Results");
	return 0;
}

void init() {
	xScales.push_back(0.5);
	xScales.push_back(0.4);
	xScales.push_back(0.3);
	xScales.push_back(0.2);
	xScales.push_back(0.1);

	yScales.push_back(0.5);
	yScales.push_back(0.4);
	yScales.push_back(0.3);
	yScales.push_back(0.2);
	yScales.push_back(0.1);

	for (int i = 0; i < 6; i++)
		angles.push_back(((double)i) * 30.0);
}


void *work(void *in) {
	arg_t *t = (arg_t *) in;
	ScoredRect rr = t->r;
	rr.score = eval(t->mat, t->rect, t->model);
	// printf("%g\n",res);
	(void)pthread_mutex_lock(&mutex);
	vRect.push_back(rr);
	work_count++;
	(void)pthread_mutex_unlock(&mutex);
	delete t;
	return NULL;
}

