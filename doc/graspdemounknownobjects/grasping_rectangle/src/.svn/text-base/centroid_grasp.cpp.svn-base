#include "feature.h"
#include <iostream>
#include "opencv/highgui.h"

IplImage* colorImg;
IplImage* background;
CvMat *bgMat;

#if USE_MOG_BG_MODEL == 1
CvGaussBGStatModelParams *modelParams;
int bg_threshold = 70;
int std_threshold = 100;
int minArea = 50;
int weight_init = 100;
int variance_init = 100; 
#else
CvFGDStatModelParams *modelParams;
int alpha1 = 100;
int alpha2 = 5;
int alpha3 = 100;
int delta = 20;
int T = 900;
int minArea = 15;
#endif

//callback function for slider, redraws
void redraw(int id) { 
  id;

#if USE_MOG_BG_MODEL == 1
  modelParams->bg_threshold  = ((float)bg_threshold)/100.0;
  modelParams->std_threshold = ((float)std_threshold)/100.0;
  modelParams->minArea       = ((float)minArea);
  modelParams->weight_init   = ((float)weight_init)/100.0;
  modelParams->variance_init = ((float)variance_init)/100.0; 
#else
  modelParams->alpha1 = ((float)alpha1) / 1000.0f;
  modelParams->alpha2 = ((float)alpha2) / 1000.0f;
  modelParams->alpha3 = ((float)alpha3) / 1000.0f;

  modelParams->delta = ((float)delta) / 10.0f;
  modelParams->T = ((float)T) / 100.0f;
  modelParams->minArea = (float)minArea;  
#endif

  cvReleaseMat(&bgMat);
  bgMat = backgroundSubtraction(colorImg, background);
  cvShowImage("img",bgMat);
  cout << "ShowImage" << endl;
}

bool SHOW_IMAGE = false;

int main(int argc, char *argv[]) {
  if (argc < 3) {
    cout << "Usage: background_subtraction imageFilename bgFilename" << endl;
    return 0;
  } 
  if (argc > 3) {
    SHOW_IMAGE = true;
  }
  initFeatures();
  modelParams = getStatParams();
  colorImg = cvLoadImage(argv[1]);
  background = cvLoadImage(argv[2]);
  //~ cvShowImage("bg",background);
  //~ cvWaitKey(0);
  //cout << "IMAGES loaded" << endl;

  bgMat = backgroundSubtraction(colorImg, background);

  if (SHOW_IMAGE) {
    cvNamedWindow("img",1);
    cvNamedWindow("mask",1);

    cvShowImage("img",bgMat);


#if USE_MOG_BG_MODEL == 1
    cvCreateTrackbar("win_size","mask",  &(modelParams->win_size),500,redraw); 
    cvCreateTrackbar("n_gauss=","mask", &(modelParams->n_gauss),100,redraw);
    cvCreateTrackbar("bg_threshold","mask", &bg_threshold,100,redraw);
    cvCreateTrackbar("std_threshold","mask", &std_threshold,100,redraw);
    cvCreateTrackbar("minArea","mask", &minArea,1000,redraw);
    cvCreateTrackbar("weight_init","mask",&weight_init,1000,redraw);
    cvCreateTrackbar("variance_init","mask",&variance_init,1000,redraw);

#else
    cvCreateTrackbar("Lc","mask",  &(modelParams->Lc  ),1000,redraw); 
    cvCreateTrackbar("N1c","mask", &(modelParams->N1c ),1000,redraw);
    cvCreateTrackbar("N2c","mask", &(modelParams->N2c ),1000,redraw);
    cvCreateTrackbar("Lcc","mask", &(modelParams->Lcc ),1000,redraw);
    cvCreateTrackbar("N1cc","mask", &(modelParams->N1cc ),1000,redraw);
    cvCreateTrackbar("N2cc","mask",&(modelParams->N2cc),1000,redraw);

    cvCreateTrackbar("alpha1","mask",&alpha1,1000,redraw);
    cvCreateTrackbar("alpha2","mask",&alpha2,1000,redraw);
    cvCreateTrackbar("alpha3","mask",&alpha3,1000,redraw);

    cvCreateTrackbar("delta","mask",&delta,1000,redraw);
    cvCreateTrackbar("T","mask",&T,100,redraw);
    cvCreateTrackbar("minArea","mask",&minArea,1000,redraw);
#endif

    cvWaitKey(0); 
  }

  IplImage bg;
  cvGetImage(bgMat, &bg);
  unsigned char* ptr;
  int totx=0, toty=0, tot=0;
  for (int i = 0; i < bg.height; i++)
  {
    ptr = bgMat->data.ptr + bgMat->step * i;
    for (int j = 0; j < bg.width; j++)
      if (ptr[j])      
      {
	tot++;
	totx+=j;
	toty+=i;
      }
  }
  for (int i=0;i<4;i++)
    printf("%d %d ",totx/tot, toty/tot);
  printf("0.0\n");
  cvReleaseMat(&bgMat);
  cvReleaseImage(&colorImg);
  cvReleaseImage(&background);
  return 0;
}
