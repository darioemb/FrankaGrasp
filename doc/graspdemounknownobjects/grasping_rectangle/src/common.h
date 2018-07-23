#pragma once
#ifndef COMMON_H
#define COMMON_H

#include <vector>
#include <istream>
#include <ostream>
#include <fstream>
#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include <string>
#include <iostream>
#include <algorithm>

using namespace std;

class Rect{
public:
	vector<double> x, y;
	bool isPos;

	Rect(double x0, double x2, double _y0, double y2) {
		x.resize(4);
		y.resize(4);
		x[0] = x[1] = x0;
		x[2] = x[3] = x2;
		y[0] = y[3] = _y0;
		y[1] = y[2] = y2;
        isPos = false;
	}
	Rect(vector<double> x_axis, vector<double> y_axis) {
		x = x_axis;
		y = y_axis;
		isPos = false;
	};
	Rect(void){
		x.resize(4);
		y.resize(4);
		isPos = false;
	};
	~Rect(void){
		x.clear();
		y.clear();
	};
	std::vector<double> getXVector() {return x;}
	std::vector<double> getYVector() {return y;}
	void setXVector(std::vector<double> x_axis) {x = x_axis;}
	void setYVector(std::vector<double> y_axis) {y = y_axis;}
	int size() {return x.size();}

	friend istream& operator>>(istream &is, Rect &r) {
		for (int i=0; i<4; i++)
			is>>r.x[i]>>r.y[i];
		return is;
	}
	friend ostream& operator<<(ostream &os, Rect &r) {
        os << r.isPos;
		for (int i=0; i<4; i++)
			os<<" "<<r.x[i]<<" "<<r.y[i];
		return os;
	}
    Rect& operator= (const Rect &r) {
        x = r.x;
        y = r.y;
        isPos = r.isPos;
        return *this;
    }
};

/*
 * The ScoredRect represents one rect and its score. It's used for sorting the rectangles by its values.
 *
 */
struct ScoredRect {
	Rect rect;
	double score;
	ScoredRect() {}
	ScoredRect(Rect r, double s) {
		rect = r;
		score = s;
	}
	/*
	 * The 'less than' is redefined for priority queue.
	 * Since we are interested in top K rects in the first step search, we use a fixed size (k) priority queue
	 * to maintain this. Thus, given k+1 rects, we need to find the one with smallest score and kick it out of
	 * queue. That's why we need the rects are sorted from the largest score to smallest.
	 */
	bool operator<(const ScoredRect &a) const{
		return score>a.score;
	}
	friend istream& operator>>(istream &is, ScoredRect &r) {
        is >> r.score;
		for (int i=0; i<4; i++)
			is>>r.rect.x[i]>>r.rect.y[i];
		return is;
	}
	friend ostream& operator<<(ostream &os, ScoredRect &r) {
        os << r.score;
		for (int i=0; i<4; i++)
			os<<" "<<r.rect.x[i]<<" "<<r.rect.y[i];
		return os;
	}
};

void loadRects(vector<Rect> &rects, const char* posFile, const char* negFile = NULL);
int getdir (string dir, vector<string> &files);
vector<Rect> ScoredRect_to_Rect(vector<ScoredRect> sr);

#endif // COMMON_H
