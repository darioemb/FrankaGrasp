#include "feature.h"
#include <limits.h>


MODEL *readAndInitModel(const char *filename)
{
  MODEL *model;
	char buff[200];
	sprintf(buff,"%s",filename);
  model = read_model(buff);
  if (model->kernel_parm.kernel_type == 0)
  {                             // linear kernel
    add_weight_vector_to_linear_model(model);
  }
  // printf("Model loaded\n");
  return model;
}

#if USE_MOG_BG_MODEL == 1
CvGaussBGStatModelParams *statModelParams;
CvGaussBGStatModelParams *getStatParams() { return statModelParams; }
#else
CvFGDStatModelParams *statModelParams;
CvFGDStatModelParams *getStatParams() { return statModelParams; }
#endif

bool isFeaturesInit = false;
void initFeatures() {
#if USE_MOG_BG_MODEL == 1
    statModelParams = new CvGaussBGStatModelParams;
    statModelParams->win_size=20;
    statModelParams->n_gauss=10;
    statModelParams->bg_threshold=0.7;
    statModelParams->std_threshold=0.8;
    statModelParams->minArea=50;
    statModelParams->weight_init=1;
    statModelParams->variance_init=1; 
#else
    statModelParams = new CvFGDStatModelParams;
    statModelParams->Lc = 128;                        /* Quantized levels per 'color' component. Power of two, typically 32, 64 or 128.                               */
    statModelParams->N1c = 15;                         /* Number of color vectors used to model normal background color variation at a given pixel.                    */
    statModelParams->N2c = 25;                         /* Number of color vectors retained at given pixel.  Must be > N1c, typically ~ 5/3 of N1c.                     */
                                            /* Used to allow the first N1c vectors to adapt over time to changing background.                               */
                                            
    statModelParams->Lcc = 64;                         /* Quantized levels per 'color co-occurrence' component.  Power of two, typically 16, 32 or 64.                 */
    statModelParams->N1cc = 25;                        /* Number of color co-occurrence vectors used to model normal background color variation at a given pixel.      */
    statModelParams->N2cc = 40;                        /* Number of color co-occurrence vectors retained at given pixel.  Must be > N1cc, typically ~ 5/3 of N1cc.     */
                                            /* Used to allow the first N1cc vectors to adapt over time to changing background.                              */
                                            
    statModelParams->is_obj_without_holes = true;        /* If TRUE we ignore holes within foreground blobs. Defaults to TRUE.                                           */
    statModelParams->perform_morphing = 1;            /* Number of erode-dilate-erode foreground-blob cleanup iterations.                                             */
                                            /* These erase one-pixel junk blobs and merge almost-touching blobs. Default value is 1.                        */
                                            
    statModelParams->alpha1 = 0.1f;                      /* How quickly we forget old background pixel values seen.  Typically set to 0.1                                */
    statModelParams->alpha2 = 0.005f;                      /* "Controls speed of feature learning". Depends on T. Typical value circa 0.005.                               */
    statModelParams->alpha3 = 0.1f;                      /* Alternate to alpha2, used (e.g.) for quicker initial convergence. Typical value 0.1.                         */
                                            
    statModelParams->delta = 2;                       /* Affects color and color co-occurrence quantization, typically set to 2.                                      */
    statModelParams->T = 0.9f;                           /* "A percentage value which determines when new features can be recognized as new background." (Typically 0.9).*/
    statModelParams->minArea = 15.f;                     /* Discard foreground blobs whose bounding box is smaller than this threshold.                                  */
#endif        
    isFeaturesInit = true;
}

CvMat *backgroundSubtraction(IplImage *fore, IplImage *back) {
	if (!isFeaturesInit) initFeatures();
    
    CvMat *mask = cvCreateMat(fore->height, fore->width, CV_8UC1);
	cvSetZero(mask);
	
	IplImage *ycbcrImgFore, *yFore, *cbFore, *crFore;

	ycbcrImgFore = cvCreateImage(cvSize(fore->width, fore->height), fore->depth, fore->nChannels);
	//~ yFore = cvCreateImage(cvSize(fore->width, fore->height), fore->depth, 1);
	//~ cbFore = cvCreateImage(cvSize(fore->width, fore->height), fore->depth, 1);
	//~ crFore = cvCreateImage(cvSize(fore->width, fore->height), fore->depth, 1);
	cvCvtColor(fore, ycbcrImgFore, CV_RGB2YCrCb);
	cvSmooth(ycbcrImgFore,ycbcrImgFore,CV_GAUSSIAN,11,11);
	//~ cvShowImage("yy",ycbcrImgFore);
	//~ cvWaitKey(0);
	//~ cvSplit(ycbcrImgFore, yFore, cbFore, crFore, NULL);
    
	IplImage *ycbcrImgBack, *yBack, *cbBack, *crBack;

	ycbcrImgBack = cvCreateImage(cvSize(back->width, back->height), back->depth, back->nChannels);
	//~ yBack = cvCreateImage(cvSize(back->width, back->height), back->depth, 1);
	//~ cbBack = cvCreateImage(cvSize(back->width, back->height), back->depth, 1);
	//~ crBack = cvCreateImage(cvSize(back->width, back->height), back->depth, 1);
	cvCvtColor(back, ycbcrImgBack, CV_RGB2YCrCb);
	cvSmooth(ycbcrImgBack,ycbcrImgBack,CV_GAUSSIAN,11,11);
	//~ cvShowImage("yy",ycbcrImgBack);
	//~ cvWaitKey(0);
	//~ cvSplit(ycbcrImgBack, yBack, cbBack, crBack, NULL);
	
#if USE_MOG_BG_MODEL == 1
	//create BG model
	CvBGStatModel* bg_model = cvCreateGaussianBGModel( ycbcrImgBack, statModelParams );
#else
	//create BG model
    cout << statModelParams->Lc << endl;
	CvBGStatModel* bg_model = cvCreateFGDStatModel( ycbcrImgBack, statModelParams );
#endif

    cvUpdateBGStatModel( ycbcrImgFore, bg_model );
    
	//~ cvShowImage("BG", bg_model->background);
	//~ cvShowImage("FG", bg_model->foreground);
	//~ cvWaitKey(0);
	//~ cout << bg_model->foreground->nChannels << endl;
	//~ cout << bg_model->foreground->depth << endl;
	cvCopy(bg_model->foreground,mask);
	cvReleaseBGStatModel(&bg_model);
	/*	
	CvScalar sF, sB, s;
	s.val[0] = 255;
	for (int y=0;y<fore->height;y++) {
		for (int x=0;x<fore->width;x++) {
			//~ sF = cvGet2D(fore,y,x);
			//~ sB = cvGet2D(back,y,x);
			if ( 
				fabs(cvGet2D(yFore,y,x).val[0] - cvGet2D(yBack,y,x).val[0]) > 20 //||
				//~ fabs(cvGet2D(cbFore,y,x).val[0] - cvGet2D(cbBack,y,x).val[0]) > 10 ||
				//~ fabs(cvGet2D(crFore,y,x).val[0] - cvGet2D(crBack,y,x).val[0]) > 10
				) {
					cvSet2D(mask, y, x, s);
			}
		}
	}
	*/
	cvErode(mask,mask,NULL,2);
	cvDilate(mask,mask,NULL,12);
	
	//~ cvReleaseImage(&yFore);
	//~ cvReleaseImage(&cbFore);
	//~ cvReleaseImage(&crFore);
	
	//~ cvReleaseImage(&yBack);
	//~ cvReleaseImage(&cbBack);
	//~ cvReleaseImage(&crBack);
	cvReleaseImage(&ycbcrImgFore);
	cvReleaseImage(&ycbcrImgBack);
	//~ cvShowImage("fore",fore);
	//~ cvShowImage("subtracted",mask);
	return mask;
}

IplImage *loadAndConvertImage(const char *filename, int cropX)
{
  if (cropX < 0)
    cropX = 0;
  IplImage *img1 = cvLoadImage(filename, CV_LOAD_IMAGE_GRAYSCALE);
  cvSetImageROI(img1, cvRect(cropX, 0, img1->width - cropX, img1->height));
  IplImage *img =
    cvCreateImage(cvSize(img1->width - cropX, img1->height), IPL_DEPTH_32F,
                  img1->nChannels);
  cvConvertScale(img1, img, 1.0 / 255.0);
  // cvCvtColor
  // printf("depth: %d w: %d h: %d\n", img->depth, img->width, img->height);
  cvReleaseImage(&img1);
  return img;
}

double eval(CvMat * mat, CvRect rect, MODEL * model)
{
  DOC *doc = extractFeature(mat, rect);
  // printf("Feature extracted\n");
  double dist;
  if (model->kernel_parm.kernel_type == 0)
  {                             // linear kernel
    dist = classify_example_linear(model, doc);
    free_example(doc, 1);
  }
  else
  {                             // non-linear kernel
    dist = classify_example(model, doc);
    free_example(doc, 1);
  }
  // printf("PREDICTION: %.8g\n",dist);
  return dist;
}

double eval(DOC* doc, MODEL* model) {
  double dist;
  if (model->kernel_parm.kernel_type == 0)
  {                             // linear kernel
    dist = classify_example_linear(model, doc);
    free_example(doc, 1);
  }
  else
  {                             // non-linear kernel
    dist = classify_example(model, doc);
    free_example(doc, 1);
  }
  // printf("PREDICTION: %.8g\n",dist);
  return dist;
}

DOC *vectorToDoc(vector<float> vec) {
  WORD *w = (WORD *) malloc(sizeof(WORD) * (vec.size() + 1));
  if (w == NULL)
    printf("MALLOC failed at extractFesture!\n");
  for (int i = 0; i < vec.size(); i++)
	{
		w[i].wnum = (long)(i+1);
		w[i].weight = vec[i];
	}
  w[vec.size()].wnum = 0;
  w[vec.size()].weight = 0.0;;
  DOC *doc = create_example(-1, 0, 0, 0.0, create_svector(w, "", 1.0));;
  free(w);
  return doc;
}

DOC *extractFeature(CvMat * mat, CvRect rect)
{
  WORD *w = (WORD *) malloc(sizeof(WORD) * (rect.width * rect.height + 1));
  if (w == NULL)
    printf("MALLOC failed at extractFesture!\n");
  float fmin = 1.0, fmax = 0.0;
  for (int i = 0; i < rect.width; i++)
  {
    for (int j = 0; j < rect.height; j++)
    {
      w[i * rect.height + j].wnum = (long)(i * rect.height + j + 1);
      w[i * rect.height + j].weight =
        (float)(cvmGet(mat, j + rect.y, i + rect.x));
      // printf("CHECK: %d %d %ld
      // %f\n",i,j,w[i*rect.height+j].wnum,w[i*rect.height+j].weight);
      if (w[i * rect.height + j].weight < fmin)
        fmin = w[i * rect.height + j].weight;
      if (w[i * rect.height + j].weight > fmax)
        fmax = w[i * rect.height + j].weight;
    }
  }
  // printf("minmax: %f %f\n",fmin,fmax);
  if (fmin == fmax)
  {
    fmin = 1 - fmin;
    fmax = 1 + fmin;
  }
  for (int i = 0; i < rect.width * rect.height; i++)
    w[i].weight = (w[i].weight - fmin) / (fmax - fmin);
  w[rect.width * rect.height].wnum = 0;
  w[rect.width * rect.height].weight = 0.0;;
  DOC *doc = create_example(-1, 0, 0, 0.0, create_svector(w, "", 1.0));;
  free(w);
  return doc;
}

void extractRect(CvArr *mat, Rect r, IplImage* &result) {
  CvSize size = cvGetSize(mat);
  IplImage tmp;
  IplImage *img = cvGetImage(mat,&tmp);
  int xmin = INT_MAX, xmax = INT_MIN;
  int ymin = INT_MAX, ymax = INT_MIN;
  double xbar = 0, ybar = 0;
  int border = BORDER_SIZE;
  for (int i=0;i<r.size();i++) {
      if (xmin > r.x[i])
          xmin = r.x[i];
      if (xmax < r.x[i])
          xmax = r.x[i];
      if (ymin > r.y[i])
          ymin = r.y[i];
      if (ymax < r.y[i])
          ymax = r.y[i];
      xbar += r.x[i];
      ybar += r.y[i];
  }
  xbar /= r.size();
  ybar /= r.size();
  if (xmin - border < 0)
    border = xmin;
  if (xmax + border >= size.width)
    border = size.width-xmax;
  if (ymin - border < 0)
    border = ymin;
  if (ymax + border >= size.height)
    border = size.height-ymax;
  xmin = xmin-border;
  xmax = xmax+border;
  ymin = ymin-border;
  ymax = ymax+border;
  cvSetImageROI(img,cvRect(xmin,ymin,xmax-xmin,ymax-ymin));
  IplImage *cropped = imcrop(img);
  cvResetImageROI(img);
  double rad = atan2(r.y[3]-r.y[0],r.x[3]-r.x[0]);
  double angle = rad / CV_PI * 180.0;
  IplImage *rotated = imrotate(cropped,angle);
  //CvPoint  box[]={r.x[0],r.y[0], r.x[1],r.y[1], r.x[2],r.y[2], r.x[3],r.y[3]};
  //CvPoint* boxArr[]={box};
  //int nPts[] = {4};
  //cvPolyLine(img, boxArr, nPts, 1, 1, cvScalar(0,255,255), 1);
  
  
  CvMat *pts = cvCreateMat(3,4,CV_32FC1);
  for (int i=0;i<4;i++) {
    cvmSet(pts,0,i,r.x[i]-xbar);
    cvmSet(pts,1,i,r.y[i]-ybar);
    cvmSet(pts,2,i,1);
  }
  CvMat *rotMatrix = cvCreateMat(2, 3, CV_32FC1);
  cv2DRotationMatrix(cvPoint2D32f(0.0,0.0),angle, 1.0, rotMatrix);
  CvMat *rotatedPts = cvCreateMat(2,4,CV_32FC1);
  cvGEMM(rotMatrix,pts,1.0,NULL,0.0,rotatedPts);
  for (int i=0;i<4;i++) {
    cvmSet(rotatedPts,0,i,cvmGet(rotatedPts,0,i)+rotated->width/2);
    cvmSet(rotatedPts,1,i,cvmGet(rotatedPts,1,i)+rotated->height/2);
    //box[i].x = cvmGet(rotatedPts,0,i);
    //box[i].y = cvmGet(rotatedPts,1,i);
    //printf("%d %d\n",box[i].x, box[i].y);
  }
  //printf("boxSize: %d %d\n",rotated->width, rotated->height);
  //cvPolyLine(rotated, boxArr, nPts, 1, 1, cvScalar(255,255,255), 1);
  
  //cvNamedWindow("ER");
  //cvShowImage("ER",img);
  //cvWaitKey(0);
  //cvShowImage("ER",cropped);
  //cvWaitKey(0);
  //cvShowImage("ER",rotated);
  //cvWaitKey(0);
  
  xmin = INT_MAX; xmax = INT_MIN;
  ymin = INT_MAX; ymax = INT_MIN;
  for (int i=0;i<r.size();i++) {
    int x = cvmGet(rotatedPts,0,i);
    int y = cvmGet(rotatedPts,1,i);
      if (xmin > x)
          xmin = x;
      if (xmax < x)
          xmax = x;
      if (ymin > y)
          ymin = y;
      if (ymax < y)
          ymax = y;
  }
  cvSetImageROI(rotated,cvRect(xmin,ymin,xmax-xmin,ymax-ymin));
  result = imcrop(rotated);
  //cvShowImage("ER",result);
  //cvWaitKey(0);
  //cvReleaseImage(&img);
  cvReleaseImage(&cropped);
  cvReleaseImage(&rotated);
  cvReleaseMat(&pts);
  cvReleaseMat(&rotMatrix);
  cvReleaseMat(&rotatedPts);
}

IplImage *imcrop(IplImage *img) {
    CvRect roi = cvGetImageROI(img);
    IplImage *res = cvCreateImage(cvSize(roi.width,roi.height),img->depth, img->nChannels);
    cvCopy(img,res);
    return res;
}

IplImage *imrotate(CvMat *mat, double angle, double magic)
{
  IplImage tmp;
  IplImage *img = cvGetImage(mat,&tmp);
  IplImage *rotated = imrotate(img,angle,magic);
	return rotated;
}

IplImage *imresize(IplImage * img, double xScale, double yScale)
{
  IplImage *dst =
    cvCreateImage(cvSize(img->width * xScale, img->height * yScale),
                  img->depth, img->nChannels);
  cvResize(img, dst, CV_INTER_CUBIC);
  return dst;
}

// Rotates the image without cropping
IplImage *imrotate(IplImage * img, double angle, double magic)
{
  double rad = angle / 180.0 * CV_PI;
  // Allocate enough space to hold the entire rotated image
  /* IplImage *dst = cvCreateImage(cvSize (img->width * abs(cos(rad)) +
     img->height * abs(sin(rad)), img->height * abs(cos(rad)) + img->width *
     abs(sin(rad))), img->depth, img->nChannels); */
  IplImage *dst = cvCreateImage(cvSize(img->width + img->height,
                                       img->width + img->height),
                                img->depth, img->nChannels);
  cvSet(dst,cvScalarAll(magic));
  // Shifts the image to the center of the allocated space
  cvSetImageROI(dst,
                cvRect((dst->width - img->width) / 2,
                       (dst->height - img->height) / 2, img->width,
                       img->height));
  cvResetImageROI(img);
  cvCopy(img, dst);
  cvResetImageROI(dst);
  // Rotate the image
  CvMat *map_matrix = cvCreateMat(2, 3, CV_32FC1);
  CvPoint2D32f center = cvPoint2D32f(dst->width / 2.0, dst->height / 2.0);
  cv2DRotationMatrix(center, angle, 1.0, map_matrix);
  IplImage *dst2 =
    cvCreateImage(cvSize(dst->width, dst->height), dst->depth, dst->nChannels);
  cvWarpAffine(dst, dst2, map_matrix, CV_INTER_LINEAR + CV_WARP_FILL_OUTLIERS,
               cvScalarAll(magic));
  IplImage *dst3 =
    cvCreateImage(cvSize
                  (img->width * abs(cos(rad)) + img->height * abs(sin(rad)),
                   img->height * abs(cos(rad)) + img->width * abs(sin(rad))),
                  img->depth, img->nChannels);
  cvSetImageROI(dst2,
                cvRect((dst2->width - dst3->width) / 2,
                       (dst2->height - dst3->height) / 2, dst3->width,
                       dst3->height));
  cvCopy(dst2, dst3);
  cvReleaseMat(&map_matrix);
  cvReleaseImage(&dst);
  cvReleaseImage(&dst2);
  return dst3;
}


// WARNING: Overwrites cloud!
void transform_to_global_frame(pcl::PointCloud<pcl::PointXYZ> &cloud,
    const vector<float> t)
{
  if (t.size() != 12) {
    printf("ERROR: t_matrix needs to have 12 elements\n");
    return;
  }
  for (unsigned int i=0;i<cloud.size();i++) {
    float x,y,z;
    x = cloud.points[i].x;
    y = cloud.points[i].y;
    z = cloud.points[i].z;
    cloud.points[i].x = t[0]*x + t[1]*y + t[2]*z + t[3];
    cloud.points[i].y = t[4]*x + t[5]*y + t[6]*z + t[7];
    cloud.points[i].z = t[8]*x + t[9]*y + t[10]*z + t[11];
  }
}

void loadPCDFile(const char* filename, pcl::PointCloud<pcl::PointXYZ> &cloud, vector<int> &indices) {
    sensor_msgs::PointCloud2 pc;
    pcl::io::loadPCDFile(string(filename),pc);
    int indexOffset;
    // find index offset
    for (unsigned int i = 0; i < pc.fields.size(); i++)
    {
      if (pc.fields[i].name.compare("index") == 0)
      {
        indexOffset = pc.fields[i].offset;
        break;
      }
    }
    indices.resize(pc.width*pc.height);
    
  for (unsigned int i = 0; i < pc.width * pc.height; i++)
  {
    indices[i] = *((int*)&pc.data[i * pc.point_step + indexOffset]);
    //printf("%d: %d\n",i,indices[i]);
  }
  pcl::fromROSMsg(pc,cloud);
}

void loadAnglesFile(const char* filename, vector<float> &joint_angles, vector<float> &t_matrix) {
    ifstream f;
    float temp;
    f.open(filename);
    joint_angles.resize(6);
    t_matrix.resize(12);
    for (int i=0;i<6;i++) {
        //printf("%d:",i);
        f >> temp;
        joint_angles[i] = temp;
    }
    
    for (int i=0;i<12;i++) {
        f >> temp;
        t_matrix[i] = temp;
    }
    f.close();
    return;
}
void calculateNormals(pcl::PointCloud<pcl::PointXYZ> &cloud, pcl::PointCloud<pcl::Normal> &normals,
        int normals_k_neighbors) {
    pcl::NormalEstimation < pcl::PointXYZ, pcl::Normal > n3d_;
    pcl::KdTree < pcl::PointXYZ >::Ptr normals_tree_ =
      boost::make_shared < pcl::KdTreeFLANN < pcl::PointXYZ > >();

    // Normal estimation parameters
    n3d_.setKSearch(normals_k_neighbors);
    // n3d_.setRadiusSearch (0.015);
    n3d_.setSearchMethod(normals_tree_);
    
    // ---[ Estimate the point normals
    // n3d_.setSearchSurface (cloud);
    n3d_.setInputCloud(boost::make_shared < pcl::PointCloud < pcl::PointXYZ >
                       >(cloud));
    n3d_.compute(normals);
}
void calculateFPFH(pcl::PointCloud<pcl::PointXYZ> &cloud, pcl::PointCloud<pcl::Normal> &normals,
        pcl::PointCloud<pcl::FPFHSignature33> &fpfh, int k_neighbors) {
    pcl::FPFHEstimation < pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33 > fpfh_;
    pcl::KdTree < pcl::PointXYZ >::Ptr normals_tree_ =
      boost::make_shared < pcl::KdTreeFLANN < pcl::PointXYZ > >();

    // fpfh_.setRadiusSearch (0.015);
    fpfh_.setSearchMethod(normals_tree_);
    // Normal estimation parameters
    // fpfh_.setKSearch(k_neighbors);
    fpfh_.setRadiusSearch (0.01);
    
    // ---[ Estimate the point normals
    // fpfh_.setSearchSurface (cloud);
    fpfh_.setInputCloud(boost::make_shared < pcl::PointCloud < pcl::PointXYZ >
                       >(cloud));
    fpfh_.setInputNormals(boost::make_shared < pcl::PointCloud < pcl::Normal >
                       >(normals));
    fpfh_.compute(fpfh);
}


vector<CvMat*> convertXYZToCvMat(const vector<int> indices, const pcl::PointCloud<pcl::PointXYZ> cloud) {
    if (indices.size() != cloud.size()) {
        printf("ERROR! indices and cloud should be of the same size!\n");
        return vector<CvMat*>();
    }
    vector<CvMat*> mat;
    mat.resize(4);
    mat[0] = cvCreateMat(IMAGE_HEIGHT,IMAGE_WIDTH,CV_32FC1);
    mat[1] = cvCreateMat(IMAGE_HEIGHT,IMAGE_WIDTH,CV_32FC1);
    mat[2] = cvCreateMat(IMAGE_HEIGHT,IMAGE_WIDTH,CV_32FC1);
    mat[3] = cvCreateMat(IMAGE_HEIGHT,IMAGE_WIDTH,CV_8UC1); // Mask
    cvSetZero(mat[0]);
    cvSetZero(mat[1]);
    cvSetZero(mat[2]);
    cvSetZero(mat[3]);
    CvScalar s; s.val[0] = 1;
    for (int i=0;i<cloud.size();i++) {
        cvmSet(mat[0],(int)(indices[i]/IMAGE_WIDTH),indices[i]%IMAGE_WIDTH,cloud.points[i].x);
        cvmSet(mat[1],(int)(indices[i]/IMAGE_WIDTH),indices[i]%IMAGE_WIDTH,cloud.points[i].y);
        cvmSet(mat[2],(int)(indices[i]/IMAGE_WIDTH),indices[i]%IMAGE_WIDTH,cloud.points[i].z);
        cvSet2D(mat[3],(int)(indices[i]/IMAGE_WIDTH),indices[i]%IMAGE_WIDTH,s);
    }
    return mat;
}
vector<CvMat*> convertNormalsToCvMat(const vector<int> indices, const pcl::PointCloud<pcl::Normal> normals, bool orientNormals) {
    if (indices.size() != normals.size()) {
        printf("ERROR! indices and normals should be of the same size!\n");
        return vector<CvMat*>();
    }
    vector<CvMat*> mat;
    mat.resize(5);
    mat[0] = cvCreateMat(IMAGE_HEIGHT,IMAGE_WIDTH,CV_32FC1);
    mat[1] = cvCreateMat(IMAGE_HEIGHT,IMAGE_WIDTH,CV_32FC1);
    mat[2] = cvCreateMat(IMAGE_HEIGHT,IMAGE_WIDTH,CV_32FC1);
    mat[3] = cvCreateMat(IMAGE_HEIGHT,IMAGE_WIDTH,CV_32FC1);
    mat[4] = cvCreateMat(IMAGE_HEIGHT,IMAGE_WIDTH,CV_8UC1);
    cvSetZero(mat[0]);
    cvSetZero(mat[1]);
    cvSetZero(mat[2]);
    cvSetZero(mat[3]);
    cvSetZero(mat[4]);
    CvScalar s; s.val[0] = 1;
    for (int i=0;i<normals.size();i++) {
        if (orientNormals && normals.points[i].normal[2] < 0) {
            cvmSet(mat[0],(int)(indices[i]/IMAGE_WIDTH),indices[i]%IMAGE_WIDTH,-normals.points[i].normal[0]);
            cvmSet(mat[1],(int)(indices[i]/IMAGE_WIDTH),indices[i]%IMAGE_WIDTH,-normals.points[i].normal[1]);
            cvmSet(mat[2],(int)(indices[i]/IMAGE_WIDTH),indices[i]%IMAGE_WIDTH,-normals.points[i].normal[2]);
            cvmSet(mat[3],(int)(indices[i]/IMAGE_WIDTH),indices[i]%IMAGE_WIDTH,normals.points[i].curvature);
            cvSet2D(mat[4],(int)(indices[i]/IMAGE_WIDTH),indices[i]%IMAGE_WIDTH,s);
        } else {
            cvmSet(mat[0],(int)(indices[i]/IMAGE_WIDTH),indices[i]%IMAGE_WIDTH,normals.points[i].normal[0]);
            cvmSet(mat[1],(int)(indices[i]/IMAGE_WIDTH),indices[i]%IMAGE_WIDTH,normals.points[i].normal[1]);
            cvmSet(mat[2],(int)(indices[i]/IMAGE_WIDTH),indices[i]%IMAGE_WIDTH,normals.points[i].normal[2]);
            cvmSet(mat[3],(int)(indices[i]/IMAGE_WIDTH),indices[i]%IMAGE_WIDTH,normals.points[i].curvature);
            cvSet2D(mat[4],(int)(indices[i]/IMAGE_WIDTH),indices[i]%IMAGE_WIDTH,s);
        }
    }
    return mat;
}

vector<CvMat*> convertFPFHToCvMat(const vector<int> indices, const pcl::PointCloud<pcl::FPFHSignature33> fpfh) {
	if (indices.size() != fpfh.size()) {
			printf("ERROR! indices and fpfh should be of the same size!\n");
			return vector<CvMat*>();
	}
	vector<CvMat*> mat;
	mat.resize(33);
	for (int i=0;i<mat.size();i++) {
		mat[i] = cvCreateMat(IMAGE_HEIGHT,IMAGE_WIDTH,CV_32FC1);
		cvSetZero(mat[i]);
	}
	CvScalar s; s.val[0] = 1;
	for (int i=0;i<fpfh.size();i++) {
		for (int j=0;j<mat.size();j++) {
			cvmSet(mat[j],(int)(indices[i]/IMAGE_WIDTH),indices[i]%IMAGE_WIDTH,fpfh.points[i].histogram[j] / 101.0);
		}
	}
	return mat;
}

vector<CvMat*> bankFilters;

// From SVMDataGenerator
void initFilters() {
	int n, m;
	double x;

	bankFilters.resize(17);
	ifstream fin("filters.txt");
	for (int i = 0; i < 17; i++) {
		fin >> n >> m;
		bankFilters[i] = cvCreateMat(n, m, CV_32FC1);
		for (int j = 0; j < n; j++)
			for (int k = 0; k < m; k++) {
				fin >> x;
				cvmSet(bankFilters[i], j, k, x);
			}
	}
}

CvMat* getFilter(int idx) {
	if (bankFilters.empty())
		initFilters();

	return idx < (int)bankFilters.size() ? bankFilters[idx] : NULL;
}

void calculateFilterBanks17(IplImage* img, vector<CvMat*>& H) {

	IplImage *ycbcrImg, *yImg, *cb, *cr;

	ycbcrImg = cvCreateImage(cvSize(img->width, img->height), img->depth, img->nChannels);
	yImg = cvCreateImage(cvSize(img->width, img->height), img->depth, 1);
	cb = cvCreateImage(cvSize(img->width, img->height), img->depth, 1);
	cr = cvCreateImage(cvSize(img->width, img->height), img->depth, 1);
	cvCvtColor(img, ycbcrImg, CV_RGB2YCrCb);
	cvSplit(ycbcrImg, yImg, cb, cr, 0);

	H.resize(17);
	for (int i = 0; i < (int)H.size(); i++) {
		H[i] = cvCreateMat(img->height, img->width, CV_32FC1);

		if (i == 9) {
			cvFilter2D(cb, H[i], getFilter(i));
		} else if (i == 10) {
			cvFilter2D(cr, H[i], getFilter(i));
		} else {
			cvFilter2D(yImg, H[i], getFilter(i));
		}

		// Texture energy
		// cvMul(H[i],H[i],H[i],1.0);
    // Abs
    cvAbsDiffS(H[i],H[i],cvScalarAll(0));
    // Sqrt
    cvPow(H[i],H[i],0.5);
    
		//normalize them
//		double minv, maxv;
//		cvMinMaxLoc(H[i], &minv, &maxv);
		cvNormalize(H[i], H[i], 0.0000001, 0.9999999, CV_MINMAX);
	}
	cvReleaseImage(&ycbcrImg);
	cvReleaseImage(&yImg);
	cvReleaseImage(&cb);
	cvReleaseImage(&cr);
}
