#ifndef STEREOIMAGE_H
#define STEREOIMAGE_H

#include "stdafx.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "triangle.h"
#include "elas.h"
#include "type.h"

class stereoImage{

public:
	struct tri_match {   
	Matcher::p_match matche;
	float t1a,t1b,t1c;
	float t2a,t2b,t2c;
	int tri_index1;
	int tri_index2;
    tri_match(Matcher::p_match matche,float t1a,float t1b,float t1c,float t2a,float t2b,float t2c,
	int tri_index1,int tri_index2):
	matche(matche),t1a(t1a),t1b(t1b),t1c(t1c),t2a(t2a),t2b(t2b),t2c(t2c),
		tri_index1(tri_index1),tri_index2(tri_index2){}
  };
	stereoImage();
	void setStereoImage(uint8_t*I1_,uint8_t*I2,int width_,int height_,int widthStep);
	//void setI1(uint8_t *I1_l,uint8_t *I1_r);
	void setD1(float* D);
	void settri1(std::vector<Elas::triangle> tri1);
	void settri2(std::vector<Elas::triangle> tri2);
	std::vector<tri_match> getTriMatch(){return triMatch;}
	void setTriMatch(std::vector<tri_match> triMatch_){triMatch=triMatch_;}
	void stereoImage::completeOneFrame();

	unsigned char* I1_l;
	unsigned char* I1_r;
	unsigned char* I2_l;
	unsigned char* I2_r;
	float* D1;
	float* D2;
	int *tri_index1;
	int *tri_index2;
	int width;
	int height;
	int widthStep;
	/*float* X1;
	float* Y1;
	float* Z1;
	float* X2;
	float* Y2;
	float* Z2;*/

	std::vector<Elas::triangle> tri1;
	std::vector<Elas::triangle> tri2;
	std::vector<Elas::support_pt> support1;
	std::vector<Elas::support_pt> support2;
	std::vector<tri_match> triMatch;
};
#endif
