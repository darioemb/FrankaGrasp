#include "common.h"
#include "feature.h"
#include "thr_pool.h"
#include "histogram.h"
#include "testUtil.h"

// using namespace std;
//char* modelfile = "model.mod";
char *modelfile = "model_poly.mod";
//char *modelfile = "model_rbf.mod";
// char* imgFile = "opencvtut1-1.png";
char *imgFile = "left0121.pgm";
char *pcdFile = "pcd0121.txt";
char *anglesFile = "pcd0121_angles.txt";
char *rectPosFile = "left0121_pos.txt";
char *rectNegFile = "left0121_neg.txt";
char *dirPrefix = "/misc/projects/grasping_data/11-6/";
//char *imgFile = "left0000.pgm";
//char *pcdFile = "pcd0000.txt";
//char *anglesFile = "pcd0000_angles.txt";
const int xCrop = 400;
const int nTop = 100;
vector < double >xScales;
vector < double >yScales;
vector < double >angles;
const double dx = 20, dy = 20;

int main(int argc, char *argv[])
{
  string dir = string(dirPrefix);
  vector<string> files = vector<string>();
  getdir(dir,files);
  // for (unsigned int i = 0;i < files.size();i++) {
      // cout << files[i] << endl;
  // }
  // return 0;
  // vector<float> joint_angles;
  // vector<float> t_matrix;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointCloud<pcl::Normal> normals;
  vector<int> indices;
  int normals_k_neighbors = 50;
  double minVal, maxVal;
  vector<CvMat*> matXYZ, matNorms;
  vector<Rect> rects;
  
  // cvMinMaxLoc(mat[2], &minVal, &maxVal, NULL, NULL, mat[4]); // Z-axis
  vector<float> hist;
  vector<float> feature_vec;
  vector<float> vecZ, vecNormZ, vecCurv;
  IplImage *m, *m2;
  CvMat *mm, *mmask; CvMat tmp, tmp2;
  float binWidth;
  int nBins = 9;
  
  for (int nFile = 125; nFile < files.size(); nFile++) {
      loadPCDFile((dir+"pcd"+files[nFile].substr(4,4)+".txt").c_str(),cloud,indices);
      calculateNormals(cloud,normals,normals_k_neighbors);
      matXYZ = convertXYZToCvMat(indices, cloud);
      matNorms = convertNormalsToCvMat(indices, normals, true);
      loadRects(rects,(dir+files[nFile].substr(0,8)+"_pos.txt").c_str(),(dir+files[nFile].substr(0,8)+"_pos.txt").c_str());
      
      // loadAnglesFile(anglesFile,joint_angles,t_matrix);
      // for (int i=0;i<rects.size();i++) {
        // cout << rects[i] << endl;
      // }
      for (int rectNum = 0;rectNum < rects.size(); rectNum++) {
          cout << (rects[rectNum].isPos?1:0) << " qid:" << (nFile+1);
          // For z values
          binWidth = 0.05;
          extractRect(matXYZ[2], rects[rectNum],m);
          mm = cvGetMat(m, &tmp);
          extractRect(matXYZ[3], rects[rectNum],m2);
          mmask = cvGetMat(m2, &tmp2);
          // Need to erode the mask due to interpolation errors from rotating
          cvErode(mmask,mmask);
          if (cvCountNonZero(mmask) == 0) {
              feature_vec.clear();
              feature_vec.resize(3*nBins*3);
              for (int i=0;i<feature_vec.size();i++)
                  feature_vec[i] = 0;
              
              for (int i=0;i<feature_vec.size();i++) {
                printf(" %d:%.3f",i+1,feature_vec[i]);
                // if (i%nBins==nBins-1) printf("\n");
              }
              printf("\n");
              continue;
          }
          
          // printf("w: %d, h: %d, t: %d\n",mm->width,mm->height,mm->width*mm->height);
          // cvMinMaxLoc(mm, &minVal, &maxVal, NULL, NULL, mmask); // Z-axis
          // printf("min: %f, max: %f\n",minVal,maxVal);
          // Get the vector for 
          getHistogramFeatureVectorDirect(mm,nBins,binWidth,mmask,vecZ,true);
          cvReleaseImage(&m);
          cvReleaseImage(&m2);
          
          // For normZ values
          binWidth = 0.1;
          extractRect(matNorms[2], rects[rectNum],m);
          mm = cvGetMat(m, &tmp);
          extractRect(matNorms[4], rects[rectNum],m2);
          mmask = cvGetMat(m2, &tmp2);
          // Need to erode the mask due to interpolation errors from rotating
          cvErode(mmask,mmask);
          // printf("w: %d, h: %d, t: %d\n",mm->width,mm->height,mm->width*mm->height);
          // cvMinMaxLoc(mm, &minVal, &maxVal, NULL, NULL, mmask); // Z-axis
          // printf("min: %f, max: %f\n",minVal,maxVal);
          // Get the vector for 
          getHistogramFeatureVectorDirect(mm,nBins,binWidth,mmask,vecNormZ,false);
          cvReleaseImage(&m);
          cvReleaseImage(&m2);
          
          // For curv values
          binWidth = 0.01;
          extractRect(matNorms[3], rects[rectNum],m);
          mm = cvGetMat(m, &tmp);
          extractRect(matNorms[4], rects[rectNum],m2);
          mmask = cvGetMat(m2, &tmp2);
          // Need to erode the mask due to interpolation errors from rotating
          cvErode(mmask,mmask);
          // printf("w: %d, h: %d, t: %d\n",mm->width,mm->height,mm->width*mm->height);
          // cvMinMaxLoc(mm, &minVal, &maxVal, NULL, NULL, mmask); // Z-axis
          // printf("min: %f, max: %f\n",minVal,maxVal);
          // Get the vector for 
          getHistogramFeatureVectorDirect(mm,nBins,binWidth,mmask,vecCurv,false);
          cvReleaseImage(&m);
          cvReleaseImage(&m2);
          
          feature_vec.clear();
          feature_vec.resize(3*nBins*3);
          
          for (int i=0;i<nBins*3;i++) {
              feature_vec[i] = vecZ[i];
              feature_vec[i+nBins*3] = vecNormZ[i];
              feature_vec[i+nBins*6] = vecCurv[i];
          }
          // feature_vec.insert(feature_vec.end(), vecZ.begin(), vecZ.end());
          // feature_vec.insert(feature_vec.end(), vecNormZ.begin(), vecNormZ.end());
          // feature_vec.insert(feature_vec.end(), vecCurv.begin(), vecCurv.end());
          
          for (int i=0;i<feature_vec.size();i++) {
            printf(" %d:%.3f",i+1,feature_vec[i]);
            // if (i%nBins==nBins-1) printf("\n");
          }
          printf("\n");
      }
  }
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
  //cvDestroyWindow("Results");
  //return 0;
}

void init()
{
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


void *work(void *in)
{
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

