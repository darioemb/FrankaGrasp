#include "Rect.h"
#include "SVMDataGenerator.h"


void Rect::setXVector(std::vector<double> x_axis){
	x = x_axis;
}

void Rect::setYVector(std::vector<double> y_axis){
	y = y_axis;
}

std::vector<double> Rect::getXVector(){
	return x;
}

std::vector<double> Rect::getYVector(){
	return y;
}

int Rect::size(){
	return x.size();
}
