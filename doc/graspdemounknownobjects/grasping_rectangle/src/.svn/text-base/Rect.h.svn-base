#pragma once
#include <istream>
#include <vector>

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
	}
	Rect(vector<double> x_axis, vector<double> y_axis) {
		x = x_axis;
		y = y_axis;
		isPos = false;
	};
	Rect(void){
		x.resize(4);
		y.resize(4);
	};
	~Rect(void){
		x.clear();
		y.clear();
	};
	std::vector<double> getXVector();
	std::vector<double> getYVector();
	void setXVector(std::vector<double> x);
	void setYVector(std::vector<double> y);
	int size();

	friend istream& operator>>(istream &is, Rect &r) {
		for (int i=0; i<4; i++)
			is>>r.x[i]>>r.y[i];
		return is;
	}
};
