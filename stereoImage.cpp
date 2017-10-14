#include "stdafx.h"
#include "stereoImage.h"
#include <stdlib.h>
#include <string.h>
#include <iostream>

using namespace std;

stereoImage::stereoImage(){}

void stereoImage::setStereoImage(uint8_t*I1_,uint8_t*I2_,int width_,int height_,int width_Step){
	if(I1_ == 0 || I2_ == 0)
	{
		cout<<"I1 or I2 error!"<<endl;
		return;
	}

	if(width_ <=0 || height_<=0)
	{
		cout<<"width or height error!"<<endl;
		return;
	}

	width = width_;
	height = height_;
	widthStep = width_Step;

	I2_l = (uint8_t*)malloc(widthStep * height * sizeof(uint8_t));
	I2_r = (uint8_t*)malloc(widthStep * height * sizeof(uint8_t));
	//D1 = (float*)malloc(width * height * sizeof(float));
	D2 =   (float*)malloc(width * height * sizeof(float));
	/*X1 = (float*)malloc(width * height * sizeof(float));
	Y1 = (float*)malloc(width * height * sizeof(float));
	Z1 = (float*)malloc(width * height * sizeof(float));
	D2 = (float*)malloc(width * height * sizeof(float));
	X2 = (float*)malloc(width * height * sizeof(float));
	Y2 = (float*)malloc(width * height * sizeof(float));
	Z2 = (float*)malloc(width * height * sizeof(float));*/
	tri_index2 = (int*)malloc(width * height * sizeof(int));
	memcpy(I2_l,I1_,width * height * sizeof(uint8_t));
	memcpy(I2_r,I2_,width * height * sizeof(uint8_t));
}

void stereoImage::completeOneFrame()
{
	if(I1_r != 0)
		free(I1_l);
	if(I1_r != 0)
		free(I1_r);
	if(D1 != 0)
		free(D1);
	/*if(X1 != 0)
		free(X1);
	if(Y1 != 0)
		free(Y1);
	if(Z1 != 0)
		free(Z1);*/
	if(tri_index1 != 0)
		free(tri_index1);
	tri1.clear();
	triMatch.clear();
	support1.clear();

	I1_r = I2_r;
	I1_l = I2_l;
	D1 = D2;
	/*X1 = X2;
	Y1 = Y2;
	Z1 = Z2;*/
	//tri1 = tri2;
	for(int i=0;i<tri2.size();i++)
		tri1.push_back(tri2[i]);
	tri_index1 = tri_index2;
	//support1 = support2;
	for(int j=0;j<support2.size();j++)
		support1.push_back(support2[j]);
	tri2.clear();
	support2.clear();
}