#pragma once
#ifndef CALC_RECT_INTERSECTION_H
#define CALC_RECT_INTERSECTION_H

#include "common.h"
#include <algorithm>
#include <vector>
#include <math.h>
#include "gpc.h"

#define PI (3.141592653589793)

using namespace std;

vector<double> calcIntersection(vector<ScoredRect> scoredRects, vector<Rect> rects, double &numTotal, double &precision);
vector<double> calcIntersection(vector<Rect> testRects, vector<Rect> rects, double &numTotal, double &precision);

double area(gpc_vertex_list list);
double area(gpc_polygon poly);

double area(gpc_vertex_list list) {
	double a = 0.0;
  int i, j = list.num_vertices-1;

  for (i=0; i<list.num_vertices; i++) {
    a += (list.vertex[j].x + list.vertex[i].x)*(list.vertex[j].y - list.vertex[i].y);
		j = i;
	}
	return fabs(a * 0.5);
}
double area(gpc_polygon poly) {
	double a = 0.0;
	for (int i=0;i<poly.num_contours;i++) {
		a += (poly.hole[i] == 0 ? 1.0 : -1.0) * area(poly.contour[i]);
	}
	return a;
}

vector<double> calcIntersection(vector<Rect> testRects, vector<Rect> rects, double &numTotal, double &precision) {
  vector<ScoredRect> scoredRects;
  scoredRects.reserve(testRects.size());
  for (int i=0;i<testRects.size();i++) {
    ScoredRect sr;
    sr.rect = testRects[i];
    sr.score = 0;
    scoredRects.push_back(sr);
  }
  return calcIntersection(scoredRects, rects, numTotal, precision);
}

//~ int calcIntersection(int filenum, ostream of) {
vector<double> calcIntersection(vector<ScoredRect> scoredRects, vector<Rect> rects, double &numTotal, double &precision) {
	vector<double> intersectionRatio;
	intersectionRatio.clear();
	
	if (rects.size() == 0) { // No positive rects!
		return intersectionRatio;
	} 
	numTotal += 1;
	
	vector<gpc_polygon> pos;  // positive polygons
	pos.reserve(rects.size());
	for (int i=0;i<rects.size();i++) {
		gpc_polygon poly;
		poly.num_contours = 1;
		poly.hole = new int[1]; poly.hole[0] = 0;
		poly.contour = new gpc_vertex_list[1];
		poly.contour[0].num_vertices = 4;
		poly.contour[0].vertex = new gpc_vertex[4];
		for (int j=0;j<4;j++) {
			poly.contour[0].vertex[j].x = rects[i].x[j];
			poly.contour[0].vertex[j].y = rects[i].y[j];
		}
		pos.push_back(poly);
	}
	
	vector<gpc_polygon> pred; // polygons predicted by algorithm
	pred.reserve(scoredRects.size());
	for (unsigned int i=0;i<scoredRects.size();i++) {
		gpc_polygon poly;
		poly.num_contours = 1;
		poly.hole = new int[1]; poly.hole[0] = 0;
		poly.contour = new gpc_vertex_list[1];
		poly.contour[0].num_vertices = 4;
		poly.contour[0].vertex = new gpc_vertex[4];
		for (int k=0;k<4;k++) {
			poly.contour[0].vertex[k].x = scoredRects[i].rect.x[k];
			poly.contour[0].vertex[k].y = scoredRects[i].rect.y[k];
		}
		pred.push_back(poly);
	}
	
	intersectionRatio.resize(pred.size());
	for (int i=0;i<pred.size();i++) {
		intersectionRatio[i] = 0.0;
		double pred_area = area(pred[i]);
		double pred_angle = atan2(
			pred[i].contour[0].vertex[3].y - pred[i].contour[0].vertex[0].y,
			pred[i].contour[0].vertex[3].x - pred[i].contour[0].vertex[0].x);
		for (int j=0;j<pos.size();j++) {
			double pos_angle = atan2(
				pos[j].contour[0].vertex[3].y - pos[j].contour[0].vertex[0].y,
				pos[j].contour[0].vertex[3].x - pos[j].contour[0].vertex[0].x);
			double ang_diff = fabs(pred_angle - pos_angle);
			if ( fabs((PI/2) - fmod(ang_diff, PI)) < (PI/3.0) ) {
				continue;
			}
			gpc_polygon poly;
			gpc_polygon_clip(GPC_INT, &pred[i], &pos[j], &poly);
			intersectionRatio[i] = fmax(intersectionRatio[i], area(poly)/pred_area);
			gpc_free_polygon(&poly);
		}
		if (i==0 && intersectionRatio[i] >= 0.5)
		  precision += 1;
	}
	
	for (int i=0;i<pred.size();i++) {
		gpc_free_polygon(&pred[i]);
	}
	for (int i=0;i<pos.size();i++) {
		gpc_free_polygon(&pos[i]);
	}
	
	return intersectionRatio;
}

#endif // CALC_RECT_INTERSECTION_H