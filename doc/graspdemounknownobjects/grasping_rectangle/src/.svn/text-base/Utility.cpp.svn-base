/*
 * Utility.cpp
 *
 *  Created on: Oct 19, 2010
 *      Author: helen
 */

#include "Utility.h"

IplImage* Utility::backGroundImage(const IplImage *image, const IplImage *background){
	
    int height = image->height;
	int depth = image->depth;
	const int width = image->width;
	int step = image->widthStep;
	int channel = image->nChannels;
	uchar *data = (uchar *)image->imageData;
	IplImage* img = cvCreateImage(cvSize(width,height),depth,channel);
	

	int bgheight = background->height;
	int bgdepth = background->depth;
	int bgwidth = background->width;
	int bgstep = background->widthStep;
	int bgchannel = background->nChannels;
	uchar *bgData = (uchar *)background->imageData;
	
	for(int i=0;i<height;i++) {
		for(int j=0;j<width;j++) {
				int index = i*step+j;
				if( data[index] == bgData[index]){
					img->imageData[index] = 255;
				}else{
					img->imageData[index] = 0;
				}
		}
	}
	

	cvDilate(img,img,NULL,1);
	
	bool check[768][1024];
		
	for (int i=0; i<height; i++)
		for (int j=0; j<width; j++)
			check[i][j] = false;
	
	int max = 0;
	int x = 0;
	int y = 0;

	for (int i=0; i<height; i++)
		for (int j=0; j<width; j++)
			
			if (!check[i][j]) {
				int num = floodfill(i,j,check,img);
				if(num>max){
				max = num;
				x = i;
				y = j;
				}
			    if(num<10000 && num >0) applyFilter(i,j,img);
			}
	
	cvDilate(img,img,NULL,1);
	cvErode(img,img,NULL,5);
    return img;
}


std::vector<double> Utility::mergeVectors(std::vector<double> v1, std::vector<double> v2){

	std::vector<double> mergedVector;
	mergedVector.reserve(v1.size()+v2.size());
    std::vector<double>::iterator iter;
	for(iter = v1.begin(); iter < v1.end(); iter++){
		mergedVector.push_back(*iter);
	}
	for(iter = v2.begin(); iter < v2.end(); iter++){
		mergedVector.push_back(*iter);
	}
	return mergedVector;
}

 Rect Utility::rectRotate(const Rect &rect, double ang, int n, int m) {
	Rect r;

	/*
	 * rect = ([cos(ang), sin(ang); -sin(ang), cos(ang)]*(rect'))';
	 */
	ang = ang/180.0*acos(-1.0);
	for(int i = 0; i < 4; i++) {
		double x = rect.x[i] - m/2.0;
		double y = rect.y[i] - n/2.0;
		r.x[i] = max(1, min(m, int(cos(ang) * x + sin(ang) * y + m/2.0)));
		r.y[i] = max(1, min(n, int(-sin(ang) * x + cos(ang) * y + n/2.0)));
	}
	return r;
}

 IplImage* Utility::imrotate(IplImage *img, double angle) {
	IplImage* dst = cvCreateImage(cvSize(img->width, img->height), img->depth, img->nChannels);
	CvMat* map_matrix = cvCreateMat(2,3,CV_32FC1);
	CvPoint2D32f center = cvPoint2D32f(img->width/2.0, img->height/2.0);
	cv2DRotationMatrix(center, angle, 1.0, map_matrix);
//	cvGetQuadrangleSubPix(img, dst, map_matrix);
	cvWarpAffine(img, dst, map_matrix, CV_WARP_FILL_OUTLIERS);
	return dst;
}

 double Utility::mean(std::vector<double> array){
	double sum = 0;
	for (int i = 0; i < array.size(); i++)
    sum = sum + array [i];

	return sum/array.size();
}

 double Utility::median(std::vector<double> array){

	std::sort(array.begin(), array.end());
	int size = array.size();
    if (size%2 == 0) return ((array[size/2]+array[size/2+1])/2);
	else return array[size/2+1];
}


 IplImage* Utility::cropImage(const IplImage *img, const CvRect region)
{
	IplImage *imageCropped;
	CvSize size;

	if (img->width <= 0 || img->height <= 0
		|| region.width <= 0 || region.height <= 0) {
		exit(1);
	}

	if (img->depth != IPL_DEPTH_8U) {
		exit(1);
	}

	cvSetImageROI((IplImage*)img, region);

	size.width = region.width;
	size.height = region.height;
	imageCropped = cvCreateImage(size, IPL_DEPTH_8U, img->nChannels);
	cvCopy(img, imageCropped);

	return imageCropped;
}

 void Utility::checkImage(IplImage *img){
	cvNamedWindow("raw image", CV_WINDOW_AUTOSIZE);
	cvShowImage("raw image", img);
	cvWaitKey(0);
}

 int Utility::floodfill(int x, int y, bool check[768][1024], IplImage* img) {
	uchar * nData = (uchar *)img->imageData;
	int step = img->widthStep;
	std::vector<int> pathx, pathy;
	pathx.push_back(x);
	pathy.push_back(y);
	int count = 0;
	check[x][y] = true;

	while (!pathx.empty()){
		
		x = pathx.back();
		y = pathy.back();
		pathx.pop_back();
		pathy.pop_back();

		for (int dx = -1; dx <=1; dx = dx+2){
			for (int dy = -1; dy <=1; dy = dy+2){

				if((x+dx > 0 && x+dx < 768) && (y+dy > 0 && y+dy < 1024)){

				   if ((!check[x+dx][y+dy]) && (nData[(x+dx)*step+(y+dy)] == 0)) {
				 	 pathx.push_back(x+dx);
				 	 pathy.push_back(y+dy);
				     count++;
				   }
				   check[x+dx][y+dy] = true;
				}
			}
		}	
		
	}
	return count;
}


 void Utility::applyFilter(int x, int y, IplImage* img){
	
	uchar * nData = (uchar *)img->imageData;
	int step = img->widthStep;
	int index = x*step+y;
	img->imageData[index] = 255;
	
	std::vector<int> pathx, pathy;
	pathx.push_back(x);
	pathy.push_back(y);
	while (!pathx.empty()){
		
		x = pathx.back();
		y = pathy.back();
		pathx.pop_back();
		pathy.pop_back();
		
		for (int dx = -1; dx <=1; dx = dx+2){
			for (int dy = -1; dy <=1; dy = dy+2){
				index = (x+dx)*step+(y+dy);
	
				if((x+dx > 0 && x+dx < 768) && (y+dy > 0 && y+dy < 1024) && (nData[index] == 0)){

					 img->imageData[index] = 255;
				 	 pathx.push_back(x+dx);
				 	 pathy.push_back(y+dy);
					
				   
				}
			}
		}	
		
	}
}
