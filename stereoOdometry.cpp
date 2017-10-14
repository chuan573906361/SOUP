#include "stdafx.h"
#include "stereoOdometry.h"
#include "type.h"
#include "cv.h"
#include "highgui.h"

using namespace std;

//#define min(a,b) a>b?b:a

visualOdometry::visualOdometry()
{
	stereoimage = new stereoImage();
	Elas::parameters param_elas;
	elas = new Elas(param_elas);
	Matcher::parameters param_match;
	matcher =new Matcher(param_match);
	planeestimate = new PlaneEstimation(stereoimage);
}

visualOdometry::~visualOdometry()
{
	free(stereoimage);
	free(elas);
	free(matcher);
	free(planeestimate);
}


void visualOdometry::run()
{
	if(stereoimage->I2_l==0||stereoimage->I2_r==0)
	{
		cout<<"no enough image in stereoimage!"<<endl;
		return;
	}

	int32_t dims_vo[3] = {stereoimage->width,stereoimage->height,stereoimage->widthStep};
    matcher->pushBack(stereoimage->I2_l,stereoimage->I2_r,dims_vo,false);
    matcher->matchFeatures(2);
    matcher->bucketFeatures(5,50,50);

    // grab matches
	
	p_matched = matcher->getMatches();

	for(int i=0;i<dims_vo[1];i++)
		for(int j=0;j<dims_vo[2];j++)
		{
			stereoimage->tri_index2[i*dims_vo[2]+j] = -1;
		}
	float *D_2 = (float*)malloc(dims_vo[2]*dims_vo[1]*sizeof(float));
	elas->process(stereoimage->I2_l,stereoimage->I2_r,stereoimage->D2,D_2,dims_vo,stereoimage->tri_index2);
	free(D_2);
	stereoimage->tri2 = elas->getTriangle();
	stereoimage->support2 = elas->getSupportPoint();
	
	if(p_matched.size() > 0)
	{
		getMatchAndTri(stereoimage,p_matched);
		planeestimate->estimateRotation();
	}
	return;
}
//////////////////////////////
///////////////merge pictures
/////////////////////////////////
void ImageMerge(IplImage* pImageA,IplImage* pImageB,IplImage*& pImageRes)
{
	assert(pImageA != NULL && pImageB != NULL);
	assert(pImageA->depth == pImageB->depth && pImageA->nChannels == pImageB->nChannels);

	if (pImageRes != NULL)
	{
	   cvReleaseImage(&pImageRes);
	   pImageRes = NULL;
	}

	CvSize size;
	size.width = pImageA->width ;//+ pImageB->width + 10;
	size.height = pImageA->height*2;
	pImageRes = cvCreateImage(size,pImageA->depth,pImageA->nChannels);

	CvRect rect = cvRect(0,0,pImageA->width,pImageA->height);
	cvSetImageROI(pImageRes,rect);
	cvRepeat(pImageA,pImageRes);
	cvResetImageROI(pImageRes);
	rect = cvRect(0,pImageA->height,pImageB->width,pImageB->height);
	cvSetImageROI(pImageRes,rect);
	cvRepeat(pImageB,pImageRes);
	cvResetImageROI(pImageRes);
}
//////////////////////////////////////////
//////////////////////////////////
void visualOdometry::getMatchAndTri(stereoImage *stereoimage,vector<Matcher::p_match> matches)
{
	vector<stereoImage::tri_match> tri_match;
	///////////////////////////////////////
	//////////////////////////////////////////
	///////////////////////////////////////////
	IplImage *image1 = cvCreateImage(cvSize(stereoimage->width,stereoimage->height),IPL_DEPTH_8U,3);
	IplImage *image2 = cvCreateImage(cvSize(stereoimage->width,stereoimage->height),IPL_DEPTH_8U,3);
	IplImage *image1_depth = cvCreateImage(cvSize(stereoimage->width,stereoimage->height),IPL_DEPTH_8U,1);
	IplImage *image2_depth = cvCreateImage(cvSize(stereoimage->width,stereoimage->height),IPL_DEPTH_8U,1);
	IplImage *merge_picture=NULL;

	for(int i=0;i<stereoimage->height;i++)
	{
		for(int j=0;j<stereoimage->width;j++)
		{
			image1->imageData[i*image1->widthStep+j*3+0]=stereoimage->I1_l[i*stereoimage->width+j];
			image1->imageData[i*image1->widthStep+j*3+1]=stereoimage->I1_l[i*stereoimage->width+j];
			image1->imageData[i*image1->widthStep+j*3+2]=stereoimage->I1_l[i*stereoimage->width+j];
			image2->imageData[i*image1->widthStep+j*3+0]=stereoimage->I2_l[i*stereoimage->width+j];
			image2->imageData[i*image1->widthStep+j*3+1]=stereoimage->I2_l[i*stereoimage->width+j];
			image2->imageData[i*image1->widthStep+j*3+2]=stereoimage->I2_l[i*stereoimage->width+j];
			image1_depth->imageData[i*image1_depth->widthStep+j]=stereoimage->D1[i*stereoimage->width+j];
			image2_depth->imageData[i*image1_depth->widthStep+j]=stereoimage->D2[i*stereoimage->width+j];
		}
	}
	ImageMerge(image1,image2,merge_picture);
		//for(int i=0;i<matches.size();i++)
		//{
		//	CvPoint centerpoint;
		//	centerpoint.x=matches[i].u1c;
		//	centerpoint.y=matches[i].v1c;
		//	cvCircle( image1, centerpoint ,0 , CV_RGB(255,0,0),1, 8, 3 );
		//	CvPoint centerpoint2;
		//	centerpoint2.x=matches[i].u1p;
		//	centerpoint2.y=matches[i].v1p;
		//	cvCircle( image2, centerpoint ,0 , CV_RGB(255,0,0),1, 8, 3 );
		//}
	///////////////////////////////////
	////////////////////////////////////////////////////////
	for(int i=0;i<matches.size();i++)
	{
		int up,vp,uc,vc;
		up = matches[i].u1p;
		vp = matches[i].v1p;
		uc = matches[i].u1c;
		vc = matches[i].v1c;
		int width = stereoimage->width;
		int height = stereoimage->height;
		int index1 = stereoimage->tri_index1[uc+vc*width];
		int index2 = stereoimage->tri_index2[up+vp*width];
		if(index1 != -1 && index2 != -1)
		{
			Elas::triangle *tri1 = &(stereoimage->tri1[index1]);
			Elas::triangle *tri2 = &(stereoimage->tri2[index2]);
			vector<point2d> tri1_points = getPointsInTri(stereoimage->I1_l,tri1,stereoimage->support1,stereoimage->D1);
			//if(index1==1077)
			//for(int i=0;i<tri1_points.size();i++)
			//{
			///*	if(tri1_points[i].v>372)
			//		cout<<"v more than 372;"<<endl;*/
			//	//cout<<tri1_points[i].u<<","<<tri1_points[i].v<<" ";
			//	double f = param_vo.f;
			//	double cu = param_vo.cu;
			//	double cv = param_vo.cv;
			//	double b = param_vo.b;
			//	int u = tri1_points[i].u;
			//	int v = tri1_points[i].v;
			//	double d = stereoimage->D1[v*width+u];
			//	if(d==-1)
			//	{
			//		
			//	}
			//	else
			//	{
			//		double X = (u-cu)*b/d;
			//		double Y = (v-cv)*b/d;
			//		double Z = f*b/d;
			//		cout<<X<<","<<Y<<","<<Z<<";"<<endl;
			//	}
			//	
			//}
			vector<point2d> tri2_points = getPointsInTri(stereoimage->I2_l,tri2,stereoimage->support2,stereoimage->D2);
			if(index2==1222)
			for(int i=0;i<tri2_points.size();i++)
			{
			/*	if(tri1_points[i].v>372)
					cout<<"v more than 372;"<<endl;*/
				//cout<<tri1_points[i].u<<","<<tri1_points[i].v<<" ";
				double f = param_vo.f;
				double cu = param_vo.cu;
				double cv = param_vo.cv;
				double b = param_vo.b;
				int u = tri2_points[i].u;
				int v = tri2_points[i].v;
				double d = stereoimage->D2[v*width+u];
				if(d==-1)
				{
					
				}
				else
				{
					double X = (u-cu)*b/d;
					double Y = (v-cv)*b/d;
					double Z = f*b/d;
					//cout<<X<<","<<Y<<","<<Z<<";"<<endl;
				}
				
			}
			if(tri1_points.size()<3 || tri2_points.size()<3)
				continue;
			////////////////////////////////////////////////////////
			//debug 画三角形
			///////////////////////////////
			//CvPoint tri[] = {stereoimage->support1[tri1->c1].u,stereoimage->support1[tri1->c1].v,
			//			stereoimage->support1[tri1->c2].u,stereoimage->support1[tri1->c2].v,
			//			stereoimage->support1[tri1->c3].u,stereoimage->support1[tri1->c3].v};
			//CvPoint* curveArr[1]={tri};  
			//int      nCurvePts[1]={3};  
			//int      nCurves=3;  
			//int      isCurveClosed=1;  
			//int      lineWidth=1;   
			//cvPolyLine(merge_picture,curveArr,nCurvePts,nCurves,isCurveClosed,cvScalar(255,0,0),lineWidth);  
			//cvPolyLine(image1_depth,curveArr,nCurvePts,nCurves,isCurveClosed,cvScalar(255,0,0),lineWidth);  
			//CvPoint centerpoint;
			//centerpoint.x=uc;
			//centerpoint.y=vc;
			//cvCircle( merge_picture, centerpoint ,0 , CV_RGB(255,0,0),1, 8, 3 );
			//CvPoint tria[] = {stereoimage->support2[tri2->c1].u,stereoimage->support2[tri2->c1].v+stereoimage->height,
			//			stereoimage->support2[tri2->c2].u,stereoimage->support2[tri2->c2].v+stereoimage->height,
			//			stereoimage->support2[tri2->c3].u,stereoimage->support2[tri2->c3].v+stereoimage->height};
			//CvPoint* curveArra[1]={tria};  
			//int      nCurvePtsa[1]={3};  
			//int      nCurvesa=3;  
			//int      isCurveCloseda=1;  
			//int      lineWidtha=1;   
			////cvPolyLine(image2,curveArra,nCurvePtsa,nCurvesa,isCurveCloseda,cvScalar(255,0,0),lineWidtha);
			//cvPolyLine(merge_picture,curveArra,nCurvePtsa,nCurvesa,isCurveCloseda,cvScalar(255,0,0),lineWidtha);
			////cvPolyLine(image2_depth,curveArra,nCurvePtsa,nCurvesa,isCurveCloseda,cvScalar(255,0,0),lineWidtha);
			//CvPoint centerpointa;
			//centerpointa.x=up;
			//centerpointa.y=vp+stereoimage->height;
			//cvCircle( merge_picture, centerpointa ,0 , CV_RGB(255,0,0),1, 8, 3 );
			//cvLine(merge_picture,centerpoint,centerpointa,CV_RGB(255,0,0),1);
			////////////////////////////////////////////
			///////////////////////////////////////
			plane *p1 = computeTrianglePlane(tri1_points,true);
			plane *p2 = computeTrianglePlane(tri2_points,false);
			if(index2 == 1222)
			{
				cout<<"plane:"<<p2->a<<","<<p2->b<<","<<p2->c<<endl;
			}
			tri_match.push_back(stereoImage::tri_match(matches[i],p1->a,p1->b,p1->c,p2->a,p2->b,p2->c,index1,index2));
		}
	}

	//cvShowImage("merge image",merge_picture);
	//cvWaitKey(0);
	if(tri_match.size()!=0)
		stereoimage->setTriMatch(tri_match);
}
#define RANSAC_NUM 20
#define RANDOM_NUM 3
visualOdometry::plane* visualOdometry::computeTrianglePlane(vector<visualOdometry::point2d> tri_point,bool pre)
{
	double f = param_vo.f;
	double cu = param_vo.cu;
	double cv = param_vo.cv;
	double b = param_vo.b;
	int width = stereoimage->widthStep;
	int height = stereoimage->height;
	int size = tri_point.size();
	if(size < 3 )
		return 0;
	float *X = (float*)malloc(size*sizeof(float));
	float *Y = (float*)malloc(size*sizeof(float));
	float *Z = (float*)malloc(size*sizeof(float));
	float *d_tem=(float*)malloc(size*sizeof(float));

	for(int i=0;i<size;i++)
	{
		float d = -1;
		int u = tri_point[i].u;
		int v = tri_point[i].v;
		if(pre)
			d = stereoimage->D1[v*width+u];
		else
			d = stereoimage->D2[v*width+u];
		if(d==-1)
		{
			X[i] = 0;
			Y[i] = 0;
			Z[i] = 0;
		}
		else
		{
			X[i] = (u-cu)*b/d;
			Y[i] = (v-cv)*b/d;
			Z[i] = f*b/d;
			d_tem[i] = d;
		}
	}
	//////////////////////////////////////////////////////
	////////////////////////////////////////////////////////最小二乘法求平面
	////////////////////////////////////////////////////////
	//Matrix A(3,3);
	//Matrix B(3,1);

	//A.zero();
	//B.zero();
	//for(int i=0;i<size;i++)
	//{
	//	float w = 10.0*d_tem[i]/12.0;
	//	//float w =1.0;
	//	/*A.val[0][0]+=X[i]*X[i];
	//	A.val[0][1]+=X[i]*Y[i];
	//	A.val[0][2]+=X[i]*Z[i];
	//	A.val[1][0]+=X[i]*Y[i];
	//	A.val[1][1]+=Y[i]*Y[i];
	//	A.val[1][2]+=Y[i]*Z[i];
	//	A.val[2][0]+=X[i]*Z[i];
	//	A.val[2][1]+=Y[i]*Z[i];
	//	A.val[2][2]+=Z[i]*Z[i];
	//	B.val[0][0]+=X[i];
	//	B.val[1][0]+=Y[i];
	//	B.val[2][0]+=Z[i];*/
	//	A.val[0][0]+=w*X[i]*X[i];
	//	A.val[0][1]+=w*X[i]*Y[i];
	//	A.val[0][2]+=w*X[i];
	//	//A.val[1][0]+=X[i]*Y[i];
	//	A.val[1][1]+=w*Y[i]*Y[i];
	//	A.val[1][2]+=w*Y[i];
	//	//A.val[2][0]+=X[i]*Z[i];
	//	//A.val[2][1]+=Y[i]*Z[i];
	//	A.val[2][2]+=w*1;
	//	B.val[0][0]+=w*X[i]*Z[i];
	//	B.val[1][0]+=w*Y[i]*Z[i];
	//	B.val[2][0]+=w*Z[i];
	//}

	//A.val[1][0]=A.val[0][1];
	//A.val[2][0]=A.val[0][2];
	//A.val[2][1]=A.val[1][2];
	//plane *result = (plane*)malloc(sizeof(plane));
	//if(B.solve(A))
	//{
	//	result->a = B.val[0][0];
	//	result->b = B.val[1][0];
	//	result->c = B.val[2][0];
	//}
	/////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////RANSAC法求平面
	///////////////////////////////////////////////////////////////////
	Matrix result_matrix(3,1);
	int inlier_num = 0;
	for(int nn=0;nn<RANSAC_NUM;nn++)
	{	
		vector<int> actives = getRandomSample_N(size,RANDOM_NUM);
		Matrix A(RANSAC_NUM,3);
		Matrix b(RANSAC_NUM,1);
		for(int i=0;i<RANDOM_NUM;i++)
		{
			A.val[i][0]=X[actives[i]];
			A.val[i][1]=Y[actives[i]];
			A.val[i][2]=1.0;
			b.val[i][0]=Z[actives[i]];
		}
		Matrix AxAT=~A*A;
		if(AxAT.inv())
		{
			Matrix result_tmp=AxAT*~A*b;
			vector<int32_t> active = getRANSACinliers(size,X,Y,Z,result_tmp);
			if(active.size()/size >0.9)
			{
				result_matrix = result_tmp;
				break;
			}
			else if(active.size() > inlier_num)
			{
				result_matrix = result_tmp;
				inlier_num = active.size();
			}
		}
	}
	plane *result = (plane*)malloc(sizeof(plane));
	result->a = result_matrix.val[0][0];
	result->b = result_matrix.val[1][0];
	result->c = result_matrix.val[2][0];

	/////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////
	//Matrix A(size,3);
	//Matrix B(size,1);
	//for(int i=0;i<size;i++)
	//{
	//	A.val[i][0]=X[i];
	//	A.val[i][1]=Y[i];
	//	A.val[i][2]=-1.0;
	//	B.val[i][0]=Z[i];
	//}

	//Matrix i(3,3);
	//i.eye();
	//i.solve(~A*A);
	//Matrix b_r = i*~A*B;

	//result->a=b_r.val[0][0];
	//result->b=b_r.val[1][0];
	//result->c=b_r.val[2][0];
	free(Z);
	free(X);
	free(Y);
	return result;
}

vector<int32_t> visualOdometry::getRANSACinliers(int size,float *x,float *y,float *z,Matrix result){
	double a = result.val[0][0];
	double b = result.val[0][1];
	double c = result.val[0][2];
	vector<int32_t> re;
	for(int i=0;i<size;i++)
	{
		if(abs(a*x[i]+b*y[i]+c-z[i])/sqrt(a*a+b*b+1) < 0.001)
			re.push_back(i);
	}
	return re;
}
vector<int32_t> visualOdometry::getRandomSample_N(int32_t N,int32_t num) {

  // init sample and totalset
  vector<int32_t> sample;
  vector<int32_t> totalset;
  
  // create vector containing all indices
  for (int32_t i=0; i<N; i++)
    totalset.push_back(i);

  // add num indices to current sample
  sample.clear();
  //cout<<"totalset size:"<<totalset.size()<<endl;
  for (int32_t i=0; i<num; i++) {
    int32_t j = rand()%totalset.size();
    sample.push_back(totalset[j]);
    totalset.erase(totalset.begin()+j);
  }
  
  // return sample
  return sample;
}
#define POINTINPANE 2
#define POINTPERCENT 0.05
vector<visualOdometry::point2d> visualOdometry::getPointsInTri(uint8_t *I,Elas::triangle *tri,vector<Elas::support_pt> p_support,float* D)
{
	vector<point2d> pointsInTri;
	pointsInTri.clear();
	int32_t c1, c2, c3;
	c1 = tri->c1;
    c2 = tri->c2;
    c3 = tri->c3;
	float plane_a=tri->t1a;
	float plane_b=tri->t1b;
	float plane_c=tri->t1c;
	int points_not_plane=0;

	float tri_u[3] ={p_support[c1].u,p_support[c2].u,p_support[c3].u};
	float tri_v[3] = {p_support[c1].v,p_support[c2].v,p_support[c3].v};

	for (uint32_t j=0; j<3; j++) {
      for (uint32_t k=0; k<j; k++) {
        if (tri_u[k]>tri_u[j]) {
          float tri_u_temp = tri_u[j]; tri_u[j] = tri_u[k]; tri_u[k] = tri_u_temp;
          float tri_v_temp = tri_v[j]; tri_v[j] = tri_v[k]; tri_v[k] = tri_v_temp;
        }
      }
    }

	float A_u = tri_u[0]; float A_v = tri_v[0];
    float B_u = tri_u[1]; float B_v = tri_v[1];
    float C_u = tri_u[2]; float C_v = tri_v[2];

	 float AB_a = 0; float AC_a = 0; float BC_a = 0;
    if ((int32_t)(A_u)!=(int32_t)(B_u)) AB_a = (A_v-B_v)/(A_u-B_u);
    if ((int32_t)(A_u)!=(int32_t)(C_u)) AC_a = (A_v-C_v)/(A_u-C_u);
    if ((int32_t)(B_u)!=(int32_t)(C_u)) BC_a = (B_v-C_v)/(B_u-C_u);
    float AB_b = A_v-AB_a*A_u;
    float AC_b = A_v-AC_a*A_u;
    float BC_b = B_v-BC_a*B_u;
        
    // first part (triangle corner A->B)
    if ((int32_t)(A_u)!=(int32_t)(B_u)) {
		for (int32_t u=max((int32_t)A_u,0); u<min((int32_t)B_u,stereoimage->width); u++){
			int32_t v_1 = (uint32_t)(AC_a*(float)u+AC_b);
			int32_t v_2 = (uint32_t)(AB_a*(float)u+AB_b);
			for (int32_t v=min(v_1,v_2); v<max(v_1,v_2); v++)
			{
				int d_plane = plane_a*u+plane_b*v+plane_c;
				int d_stereo = D[u+v*stereoimage->width];
				if(abs(d_plane-d_stereo)<=POINTINPANE)
				{
					pointsInTri.push_back(point2d(u,v));
				}
				else
				{
					points_not_plane++;
				}
			}
		}
    }

    // second part (triangle corner B->C)
    if ((int32_t)(B_u)!=(int32_t)(C_u)) {
		for (int32_t u=max((int32_t)B_u,0); u<min((int32_t)C_u,stereoimage->width); u++){
			int32_t v_1 = (uint32_t)(AC_a*(float)u+AC_b);
			int32_t v_2 = (uint32_t)(BC_a*(float)u+BC_b);
			for (int32_t v=min(v_1,v_2); v<max(v_1,v_2); v++)
			{
				int d_plane = plane_a*u+plane_b*v+plane_c;
				int d_stereo = D[u+v*stereoimage->width];
				if(abs(d_plane-d_stereo)<=POINTINPANE)
				{
					pointsInTri.push_back(point2d(u,v));
				}
				else
				{
					points_not_plane++;
				}
			}
		}
	}

	//检查是否有同样元素
	vector<point2d> points_result;
	for(int i=1;i<pointsInTri.size();i++)
	{
		int u_cur = pointsInTri[i].u;
		int v_cur = pointsInTri[i].v;
		int flag = 0;
		for(int j=0;j<i;j++)
		{
			if(pointsInTri[j].u = u_cur && pointsInTri[j].v == v_cur)
				flag = 1;
		}
		if(!flag)
			points_result.push_back(pointsInTri[i]);
	}

	if((float)points_not_plane > ((float)points_result.size())*POINTPERCENT)
	{
		points_result.clear();
	}
	return points_result;
}