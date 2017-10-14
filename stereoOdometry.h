#ifndef STEREOODOMETRY_H
#define STEREOODOMETRY_H

#include "stdafx.h"
#include <iostream>
#include "stereoImage.h"
#include "elas.h"
#include "matcher.h"
#include "planeEstimation.h"
#include "type.h"
#include "Matrix.h"

using namespace std;

class visualOdometry{
	
public:
	visualOdometry();
	~visualOdometry();
	void run();
	stereoImage* getStereoImage(){return stereoimage;}
	void setStereoImage(stereoImage * si){stereoimage=si;}
	//void getMatchAndTri(stereoImage *stereoimage,vector<Matcher::p_match> matches);
	void setCalib(float f,float cu,float cv,float b){param_vo.f=f;param_vo.b=b;param_vo.cu=cu;param_vo.cv=cv;}
	struct param_cam{
		float f;
		float cu;
		float cv;
		float b;
		param_cam(){f=1;cu=0;cv=0;b=1;}
	};
	struct point2d{
		int u;
		int v;
		point2d(int u,int v):u(u),v(v){}
	};
	struct plane{
		float a;
		float b;
		float c;
		plane(){a=0;b=0;c=0;}
	};
	PlaneEstimation *planeestimate;

private:
	std::vector<int32_t> visualOdometry::getRANSACinliers(int size,float *x,float *y,float *z,Matrix result);
	std::vector<int32_t> visualOdometry::getRandomSample_N(int32_t N,int32_t num);
	void getMatchAndTri(stereoImage *stereoimage,vector<Matcher::p_match> matches);
	std::vector<point2d> getPointsInTri(uint8_t *I,Elas::triangle *tri,vector<Elas::support_pt> p_support,float*D);
	plane *computeTrianglePlane(vector<point2d> tri_point,bool pre);
	stereoImage *stereoimage;
	Elas *elas;
	Matcher *matcher;
	std::vector<Matcher::p_match> p_matched;
	//PlaneEstimation *planeestimate;
	param_cam param_vo;
};


#endif