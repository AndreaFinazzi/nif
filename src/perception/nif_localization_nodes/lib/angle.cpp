#include <iostream>
#include <sstream>
#include <stdio.h>
#include <cmath>

int main(void){
	float x= 1;
	float y= 0;
	float x_ = -0.17;
	float y_ = +0.98;
	float dot = x*x_ + y*y_;
	float det = x*y_ -y*x_;
	float angle = atan2(det, dot);
	// angle = angle *180/M_PI;
	float mag = sqrt(pow(x_,2)+pow(y_,2));
	float x__ = mag*cos(angle);
	float y__ = mag*sin(angle);
	std::cout << "x__ : " << x__ << std::endl;
    std::cout << "y__ : " << y__ << std::endl;
	return 0;
}