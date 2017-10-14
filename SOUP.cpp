// SOUP.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include "cv.h"
#include "cxmisc.h"
#include "highgui.h"
#include "cvaux.h"
#include <vector>
#include <string>
#include <algorithm>
#include <stdio.h>
#include <ctype.h>
#include <iostream>
#include "elas.h"
#include <sstream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <string.h>
#include "filter.h"
#include "matcher.h"
#include "triangle.h"
#include "stereoOdometry.h"
#include "stereoImage.h"
#include "type.h"

int main(int argc, _TCHAR* argv[])
{
	visualOdometry *vo = new visualOdometry();
	vo->setCalib(645.24,635.959,194.129,0.570738);

	char fn1[1024];
    char fn2[1024];
	IplImage* I1_curr;
	IplImage* I2_curr;
	stereoImage *si;
	for (int32_t i=0; i<100; i++) {

      string input_dir_str = "2010_03_09_drive_0019\\2010_03_09_drive_0019";
      sprintf(fn1,"%s\\I1_%06d.png",input_dir_str.c_str(),i);
      sprintf(fn2,"%s\\I2_%06d.png",input_dir_str.c_str(),i);

      printf("Reading: I1_%06d.png, I2_%06d.png",i,i);
      cout << endl;
      I1_curr = cvLoadImage(fn1,CV_LOAD_IMAGE_GRAYSCALE);
      I2_curr = cvLoadImage(fn2,CV_LOAD_IMAGE_GRAYSCALE);
	  si = vo->getStereoImage(); 
	  si->setStereoImage((uint8_t*)I1_curr->imageData,(uint8_t*)I2_curr->imageData,I1_curr->width,I1_curr->height,I1_curr->widthStep);
	  vo->run();
	  si->completeOneFrame();
    }

	while(1);
	return 0;
}

