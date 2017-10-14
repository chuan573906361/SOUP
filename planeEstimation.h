#ifndef PLANEESTIMATION_H
#define PLANEESTIMATION_H

#include "stdafx.h"
#include <vector>
#include <stdint.h>
#include <math.h>
#include "matrix.h"
#include "matcher.h"
#include "stereoImage.h"
#include "type.h"

class PlaneEstimation {

public:

    PlaneEstimation(stereoImage *si);
	void estimateRotation();
	Matrix R_total;

	struct tria{
		float a,b,c;
		tria(float a,float b,float c):a(a),b(b),c(c){}
	};
	struct tri_match_multi{
		float t1a,t1b,t1c;
		//int num_match;
		std::vector<tria> tri_2;
		tri_match_multi(float t1a,float t1b,float t1c,std::vector<tria> tri_2):t1a(t1a),t1b(t1b),t1c(t1c),tri_2(tri_2){}
	};
private:
	std::vector<int> getTransInliers(std::vector<int> inlier,std::vector<stereoImage::tri_match> tri_match,Matrix b);
	std::vector<int32_t> PlaneEstimation::getRandomSample_N(int32_t N,int32_t num);
	void getTrans(std::vector<int> inlier,std::vector<stereoImage::tri_match> tri_match,Matrix& t,Matrix R);
	std::vector<tri_match_multi> getMultiMatch(std::vector<stereoImage::tri_match> t);
	int getRandom(int N);
//	void estimateRotation();
	int getRotationMatrix(stereoImage::tri_match tm,Matrix &R);
	std::vector<int32_t> getInlier(std::vector<stereoImage::tri_match> tm,Matrix R);
	std::vector<stereoImage::tri_match> getValidTriMatch(std::vector<stereoImage::tri_match>);
	void matrix_norm(Matrix& a);
	double getInnerProduct(Matrix a,Matrix b);
	Matrix antisymmetricMatrix(Matrix n);
	Matrix transposition(Matrix n);
    float  f,cu,cv,base;

	std::vector<Matcher::p_match> p_matched;
	stereoImage *stereoimage;
	int count;
	//Matrix R_total;
};

#endif