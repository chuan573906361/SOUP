#include "stdafx.h"
#ifndef __ELAS_H__
#define __ELAS_H__

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <vector>
#include <emmintrin.h>
#include "matcher.h"
#include "type.h"

// define fixed-width datatypes for Visual Studio projects
//#ifndef _MSC_VER
//  #include <stdint.h>
//#else
//  typedef __int8            int8_t;
//  typedef __int16           int16_t;
//  typedef __int32           int32_t;
//  typedef __int64           int64_t;
//  typedef unsigned __int8   uint8_t;
//  typedef unsigned __int16  uint16_t;
//  typedef unsigned __int32  uint32_t;
//  typedef unsigned __int64  uint64_t;
//#endif

#ifdef PROFILE
#include "timer.h"
#endif

class Elas {
  
public:
  
  enum setting {ROBOTICS,MIDDLEBURY};
  
  // parameter settings
  struct parameters {
    int32_t disp_min;               // min disparity
    int32_t disp_max;               // max disparity
    float   support_threshold;      // max. uniqueness ratio (best vs. second best support match)  最大视差唯一性百分比
    int32_t support_texture;        // min texture for support points      最小纹理支持点
    int32_t candidate_stepsize;     // step size of regular grid on which support points are matched
    int32_t incon_window_size;      // window size of inconsistent support point check
    int32_t incon_threshold;        // disparity similarity threshold for support point to be considered consistent
    int32_t incon_min_support;      // minimum number of consistent support points
    bool    add_corners;            // add support points at image corners with nearest neighbor disparities
    int32_t grid_size;              // size of neighborhood for additional support point extrapolation
    float   beta;                   // image likelihood parameter
    float   gamma;                  // prior constant
    float   sigma;                  // prior sigma
    float   sradius;                // prior sigma radius
    int32_t match_texture;          // min texture for dense matching
    int32_t lr_threshold;           // disparity threshold for left/right consistency check
    float   speckle_sim_threshold;  // similarity threshold for speckle segmentation
    int32_t speckle_size;           // maximal size of a speckle (small speckles get removed)  
    int32_t ipol_gap_width;         // interpolate small gaps (left<->right, top<->bottom)
    bool    filter_median;          // optional median filter (approximated)
    bool    filter_adaptive_mean;   // optional adaptive mean filter (approximated)
    bool    postprocess_only_left;  // saves time by not postprocessing the right image
    bool    subsampling;            // saves time by only computing disparities for each 2nd pixel
                                    // note: for this option D1 and D2 must be passed with size
                                    //       width/2 x height/2 (rounded towards zero)
    
    // constructor
	parameters (setting s=ROBOTICS) {
      
      // default settings in a robotics environment
      // (do not produce results in half-occluded areas
      //  and are a bit more robust towards lighting etc.)  //默认设置为实验环境，不能在half-occluded环境和大量光照条件下使用
      if (s==ROBOTICS) {
        disp_min              = 0;
        disp_max              = 63;
        support_threshold     = 0.85;
        support_texture       = 10;
        candidate_stepsize    = 5;
        incon_window_size     = 5;
        incon_threshold       = 5;
        incon_min_support     = 5;
        add_corners           = 0;
        grid_size             = 20;
        beta                  = 0.02;
        gamma                 = 3;
        sigma                 = 1;
        sradius               = 2;
        match_texture         = 1;                //dense matching的最小纹理
        lr_threshold          = 2;                //一致性检测阈值  modified by chuan
        speckle_sim_threshold = 2;                //删除细小片段相似性分割阈值
        speckle_size          = 200;              //删除细小片段size
        ipol_gap_width        = 7 ;                //间隙插值阈值     
        filter_median         = 0;
        filter_adaptive_mean  = 0;
        postprocess_only_left = 1;
        subsampling           = 0;


        
      // default settings for middlebury benchmark  
      // (interpolate all missing disparities)   middlebury基准，插入所有失踪的视差
      } else {
        disp_min              = 0;
        disp_max              = 63;
        support_threshold     = 0.85;
        support_texture       = 10;
        candidate_stepsize    = 5;
        incon_window_size     = 5;
        incon_threshold       = 5;
        incon_min_support     = 5;
        add_corners           = 1;
        grid_size             = 20;
        beta                  = 0.02;
        gamma                 = 5;
        sigma                 = 1;
        sradius               = 3;
        match_texture         = 0;
        lr_threshold          = 2;
        speckle_sim_threshold = 1;
        speckle_size          = 200;
        ipol_gap_width        = 5000;
        filter_median         = 1;
        filter_adaptive_mean  = 0;
        postprocess_only_left = 0;
        subsampling           = 0;
      }
    }
  };

  // constructor, input: parameters  
  Elas (parameters param) : param(param) {}

  // deconstructor
  ~Elas () {}  
  
  struct support_pt {
    int32_t u;
    int32_t v;
    int32_t d;
    support_pt(int32_t u,int32_t v,int32_t d):u(u),v(v),d(d){}
  };
   struct triangle {
    int32_t c1,c2,c3;
    float   t1a,t1b,t1c;
    float   t2a,t2b,t2c;
    triangle(int32_t c1,int32_t c2,int32_t c3):c1(c1),c2(c2),c3(c3){}
  };
private:
  
  //struct support_pt {
  //  int32_t u;
  //  int32_t v;
  //  int32_t d;
  //  support_pt(int32_t u,int32_t v,int32_t d):u(u),v(v),d(d){}
  //};

 /* struct triangle {
    int32_t c1,c2,c3;
    float   t1a,t1b,t1c;
    float   t2a,t2b,t2c;
    triangle(int32_t c1,int32_t c2,int32_t c3):c1(c1),c2(c2),c3(c3){}
  };*/

  inline uint32_t getAddressOffsetImage (const int32_t& u,const int32_t& v,const int32_t& width) {
    return v*width+u;
  }

  inline uint32_t getAddressOffsetGrid (const int32_t& x,const int32_t& y,const int32_t& d,const int32_t& width,const int32_t& disp_num) {
    return (y*width+x)*disp_num+d;
  }

  // support point functions
  void removeInconsistentSupportPoints (int16_t* D_can,int32_t D_can_width,int32_t D_can_height);
  void removeRedundantSupportPoints (int16_t* D_can,int32_t D_can_width,int32_t D_can_height,
                                     int32_t redun_max_dist, int32_t redun_threshold, bool vertical);
  void addCornerSupportPoints (std::vector<support_pt> &p_support);
  inline int16_t computeMatchingDisparity (const int32_t &u,const int32_t &v,uint8_t* I1_desc,uint8_t* I2_desc,const bool &right_image);
  std::vector<support_pt> computeSupportMatches (uint8_t* I1_desc,uint8_t* I2_desc);
  void computeSupportMatches_corner(uint8_t *I1_desc,uint8_t *I2_desc);
  std::vector<support_pt> cl_sobelandcomputeSupportMatches (uint8_t* I1_desc,uint8_t* I2_desc);
  // triangulation & grid
  std::vector<triangle> computeDelaunayTriangulation (std::vector<support_pt> p_support,int32_t right_image);
  void computeDisparityPlanes (std::vector<support_pt> p_support,std::vector<triangle> &tri,int32_t right_image);
  void createGrid (std::vector<support_pt> p_support,int32_t* disparity_grid,int32_t* grid_dims,bool right_image);

  // matching
  inline void updatePosteriorMinimum (__m128i* I2_block_addr,const int32_t &d,const int32_t &w,
                                      const __m128i &xmm1,__m128i &xmm2,int32_t &val,int32_t &min_val,int32_t &min_d);
  inline int Elas::updatePosteriorMinimum1(__m128i* I2_block_addr,const int32_t &d,const int32_t &w,
                                         const __m128i &xmm1,__m128i &xmm2,int32_t &val,int32_t &min_val,int32_t &min_d);
  inline void updatePosteriorMinimum (__m128i* I2_block_addr,const int32_t &d,
                                      const __m128i &xmm1,__m128i &xmm2,int32_t &val,int32_t &min_val,int32_t &min_d);
  inline int findMatch (int32_t &u,int32_t &v,float &plane_a,float &plane_b,float &plane_c,
                         int32_t* disparity_grid,int32_t *grid_dims,uint8_t* I1_desc,uint8_t* I2_desc,
                         int32_t *P,int32_t &plane_radius,bool &valid,bool &right_image,float* D);
  void computeDisparity (std::vector<support_pt> p_support,std::vector<triangle> tri,int32_t* disparity_grid,int32_t* grid_dims,
                         uint8_t* I1_desc,uint8_t* I2_desc,bool right_image,float* D,int *tri_index);

  // L/R consistency check
  void leftRightConsistencyCheck (float* D1,float* D2);
  
  // postprocessing
  void removeSmallSegments (float* D);
  void gapInterpolation (float* D);

  // optional postprocessing
  void adaptiveMean (float* D);
  void median (float* D);
  
  // parameter set
  parameters param;
  
  // memory aligned input images + dimensions
  uint8_t *I1,*I2;
  uint8_t *flag_image;
  int32_t width,height,bpl;
  std::vector<support_pt> p_support;
  std::vector<triangle> tri_1;
  std::vector<Matcher::maximum> maxima;

  public:

	  // matching function
  // inputs: pointers to left (I1) and right (I2) intensity image (uint8, input)
  //         pointers to left (D1) and right (D2) disparity image (float, output)
  //         dims[0] = width of I1 and I2
  //         dims[1] = height of I1 and I2
  //         dims[2] = bytes per line (often equal to width, but allowed to differ)
  //         note: D1 and D2 must be allocated before (bytes per line = width)
  //               if subsampling is not active their size is width x height,
  //               otherwise width/2 x height/2 (rounded towards zero)
  void process (uint8_t* I1,uint8_t* I2,float* D1,float* D2,const int32_t* dims,int *tri_index);//, bool bCompSuppt, vector<support_pt> p_support);
  std::vector<support_pt> pGlbSupportpoint;
  std::vector<support_pt> getSupportPoint(){return p_support;}
  std::vector<triangle> getTriangle(){return tri_1;}
  void setMaxma(std::vector<Matcher::maximum> maxima_tmp){maxima = maxima_tmp;}
  int num_cut_support;
  // profiling timer
#ifdef PROFILE
  Timer timer;
#endif
};

#endif