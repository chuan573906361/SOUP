#include "stdafx.h"
#include "planeEstimation.h"
#include "cv.h"
#include <iostream>
#include "opencv2/core/core.hpp"
#include <string>

using namespace std;

PlaneEstimation::PlaneEstimation(stereoImage *si)
{
	stereoimage = si;
}

int PlaneEstimation::getRandom(int N)
{
	int sample = 0;
	vector<int32_t> totalset;
  
  // create vector containing all indices
	for (int32_t i=0; i<N; i++)
		totalset.push_back(i);

	sample = rand()%totalset.size();
  
  // return sample
	return sample;
}

vector<int32_t> PlaneEstimation::getRandomSample_N(int32_t N,int32_t num) {

  // init sample and totalset
  vector<int32_t> sample;
  vector<int32_t> totalset;
  
  if(num>N)
  {
	cout<<"error in get random sample"<<endl;
	while(1);
  }
  // create vector containing all indices
  for (int32_t i=0; i<N; i++)
    totalset.push_back(i);

  // add num indices to current sample
  sample.clear();
  for (int32_t i=0; i<num; i++) {
    int32_t j = rand()%totalset.size();
    sample.push_back(totalset[j]);
    totalset.erase(totalset.begin()+j);
  }
  
  // return sample
  return sample;
}
vector<PlaneEstimation::tri_match_multi> PlaneEstimation::getMultiMatch(vector<stereoImage::tri_match> t)
{
	vector<tri_match_multi> result;
	vector<stereoImage::tri_match> rest;
	rest=t;
	for(int i=0;i<t.size();i++)
	{
		vector<stereoImage::tri_match> r;
		r.clear();
		stereoImage::tri_match si_temp=t[i];
		int flag=0;
		for(int j=0;j<rest.size();j++)
		{
			//int flag=0;
			stereoImage::tri_match si_in = rest[j];
			if(
				/*si_in.t1a==si_temp.t1a && 
				si_in.t1b==si_temp.t1b &&
				si_in.t1c==si_temp.t1c &&*/
				si_in.tri_index1==si_temp.tri_index1)
			{
				if(flag==0)
				{
					vector<tria> t2;
					t2.push_back(tria(si_in.t2a,si_in.t2b,si_in.t2c));
					tri_match_multi t1(si_temp.t1a,si_temp.t1b,si_temp.t1c,t2);
					result.push_back(t1);
					flag++;
				}
				else
				{
					tri_match_multi t3 = result.back();
					t3.tri_2.push_back(tria(si_in.t2a,si_in.t2b,si_in.t2c));
					result.push_back(t3);
				}
			}
			else{
				r.push_back(si_in);
			}
		}
		rest=r;
	}
	return result;
}
#define ITERATION 50
#define match_factor 0.9
void PlaneEstimation::estimateRotation()
{
	/*vector<tri_match_multi> t = getMultiMatch(stereoimage->getTriMatch());
	for(int i=0;i<t.size();i++)
	{
		cout<<"no"<<i<<":"<<endl;
		cout<<"tri1:"<<t[i].t1a<<","<<t[i].t1b<<","<<t[i].t1c<<endl;
		for(int j=0;j<t[i].tri_2.size();j++)
		{
			cout<<"tri2:"<<t[i].tri_2[j].a<<","<<t[i].tri_2[j].b<<","<<t[i].tri_2[j].c<<endl;
		}
	}

	vector<stereoImage::tri_match> t_match;
	for(int i=0;i<t.size();i++)
	{
		if(t[i].tri_2.size()>1)
		{
			for(int j=0;j<t[i].tri_2.size();j++)
			{
				tria tn_tmp = t[i].tri_2[j];
				for(int k=1;k<t[i].tri_2.size();k++)
				{
					if(tn_tmp.a==t[i].tri_2[k].a &&
						tn_tmp.b==t[i].tri_2[k].b &&
						tn_tmp.c==t[i].tri_2[k].c)
					{
						t_match.push_back(stereoImage::tri_match(stereoimage->triMatch[0].matche,t[i].t1a,t[i].t1b,t[i].t1c,t[i].tri_2[k].a,t[i].tri_2[k].b,t[i].tri_2[k].c,0,0));
						break;
						break;
					}
				}
			}
		}
	}
	for(int nn=0;nn<t_match.size();nn++)
	{
		Matrix Raa(3,3);
		getRotationMatrix(t_match[nn],Raa);
		cout<<"R:"<<Raa<<endl;
	}*/
	IplImage *image1 = cvCreateImage(cvSize(stereoimage->width,stereoimage->height),IPL_DEPTH_8U,3);
	IplImage *image2 = cvCreateImage(cvSize(stereoimage->width,stereoimage->height),IPL_DEPTH_8U,3);

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
			/*image1_depth->imageData[i*image1_depth->widthStep+j]=stereoimage->D1[i*stereoimage->width+j];
			image2_depth->imageData[i*image1_depth->widthStep+j]=stereoimage->D2[i*stereoimage->width+j];*/
		}
	}
	string filename="1.xml";
	cv::FileStorage fs(filename, cv::FileStorage::WRITE);

	vector<stereoImage::tri_match> tri_match_get = stereoimage->getTriMatch();
	vector<stereoImage::tri_match> tri_match = getValidTriMatch(tri_match_get);
	//vector<stereoImage::tri_match> tri_match = tri_match_get;
	cout<<"triangle match num:"<<tri_match.size()<<endl;
	vector<int> inlier;
	inlier.clear();
	int triMatch_size = tri_match.size();
	//Matrix R_total;
	int random_choose;
	for(int k = 0;k < ITERATION;k++)
	//for(int k = 0;k < triMatch_size;k++)
	{
		int random = getRandom(triMatch_size);

		stereoImage::tri_match random_triMatch = tri_match[random];
		
		//stereoImage::tri_match random_triMatch = tri_match[k];
		Matrix R(3,3);
		int result = getRotationMatrix(random_triMatch,R);
		if(result != 1)
		{
			cout<<"error in get rotation matrix!"<<endl;
			break;
		}
		cv::Mat r_mat(3,3,CV_32F);
		for(int i=0;i<3;i++)
			for(int j=0;j<3;j++)
				r_mat.at<float>(i,j)=R.val[i][j];
	/*	cout<<"R_in:"<<R<<endl;
		fs<<"R_in"<<r_mat;*/
		vector<int> inlier_cur = getInlier(tri_match,R);
		if(inlier_cur.size() > inlier.size())
		{
			inlier = inlier_cur;
			if(inlier.size() > match_factor*triMatch_size)
			{
				R_total = R;
				random_choose=random;
				break;
			}
			R_total = R;
			random_choose = random;
			//printf("inliers:%f\n",((float)inlier.size())/((float)triMatch_size));
		}
//IplImage *image1 = cvCreateImage(cvSize(stereoimage->width,stereoimage->height),IPL_DEPTH_8U,3);
//IplImage *image2 = cvCreateImage(cvSize(stereoimage->width,stereoimage->height),IPL_DEPTH_8U,3);
//
//for(int i=0;i<stereoimage->height;i++)
//	{
//		for(int j=0;j<stereoimage->width;j++)
//		{
//			image1->imageData[i*image1->widthStep+j*3+0]=stereoimage->I1_l[i*stereoimage->width+j];
//			image1->imageData[i*image1->widthStep+j*3+1]=stereoimage->I1_l[i*stereoimage->width+j];
//			image1->imageData[i*image1->widthStep+j*3+2]=stereoimage->I1_l[i*stereoimage->width+j];
//			image2->imageData[i*image1->widthStep+j*3+0]=stereoimage->I2_l[i*stereoimage->width+j];
//			image2->imageData[i*image1->widthStep+j*3+1]=stereoimage->I2_l[i*stereoimage->width+j];
//			image2->imageData[i*image1->widthStep+j*3+2]=stereoimage->I2_l[i*stereoimage->width+j];
//			/*image1_depth->imageData[i*image1_depth->widthStep+j]=stereoimage->D1[i*stereoimage->width+j];
//			image2_depth->imageData[i*image1_depth->widthStep+j]=stereoimage->D2[i*stereoimage->width+j];*/
//		}
//	}
//int tri_num=tri_match[random].tri_index1;
//CvPoint tri[] = {stereoimage->support1[stereoimage->tri1[tri_num].c1].u,stereoimage->support1[stereoimage->tri1[tri_num].c1].v,
//						stereoimage->support1[stereoimage->tri1[tri_num].c2].u,stereoimage->support1[stereoimage->tri1[tri_num].c2].v,
//						stereoimage->support1[stereoimage->tri1[tri_num].c3].u,stereoimage->support1[stereoimage->tri1[tri_num].c3].v};
//	CvPoint* curveArr[1]={tri};  
//	int      nCurvePts[1]={3};  
//	int      nCurves=3;  
//	int      isCurveClosed=1;  
//	int      lineWidth=1;   
//	cvPolyLine(image1,curveArr,nCurvePts,nCurves,isCurveClosed,cvScalar(255,0,0),lineWidth);
//	int tri_num1=tri_match[random].tri_index2;
//	CvPoint tri1[] = {stereoimage->support2[stereoimage->tri2[tri_num1].c1].u,stereoimage->support2[stereoimage->tri2[tri_num1].c1].v,
//						stereoimage->support2[stereoimage->tri2[tri_num1].c2].u,stereoimage->support2[stereoimage->tri2[tri_num1].c2].v,
//						stereoimage->support2[stereoimage->tri2[tri_num1].c3].u,stereoimage->support2[stereoimage->tri2[tri_num1].c3].v};
//	CvPoint* curveArr1[1]={tri1};  
//int      nCurvePts1[1]={3};  
//int      nCurves1=3;  
//int      isCurveClosed1=1;  
//int      lineWidth1=1;   
//cvPolyLine(image2,curveArr1,nCurvePts1,nCurves1,isCurveClosed1,cvScalar(255,0,0),lineWidth1);
//cout<<"R:"<<R<<endl;
//cvReleaseImage(&image1);
//cvReleaseImage(&image2);
	}
	printf("inliers:%f\n",((float)inlier.size()-1.0)/((float)triMatch_size));
	cout<<"R:"<<R_total<<endl;
	//////////////////////////////////////////////
	////测试
	//////////////////////////////////////////////
	//IplImage *image1 = cvCreateImage(cvSize(stereoimage->width,stereoimage->height),IPL_DEPTH_8U,3);
	//IplImage *image2 = cvCreateImage(cvSize(stereoimage->width,stereoimage->height),IPL_DEPTH_8U,3);
	///*IplImage *image1_depth = cvCreateImage(cvSize(stereoimage->width,stereoimage->height),IPL_DEPTH_8U,1);
	//IplImage *image2_depth = cvCreateImage(cvSize(stereoimage->width,stereoimage->height),IPL_DEPTH_8U,1);
	//IplImage *merge_picture=NULL;*/

	//for(int i=0;i<stereoimage->height;i++)
	//{
	//	for(int j=0;j<stereoimage->width;j++)
	//	{
	//		image1->imageData[i*image1->widthStep+j*3+0]=stereoimage->I1_l[i*stereoimage->width+j];
	//		image1->imageData[i*image1->widthStep+j*3+1]=stereoimage->I1_l[i*stereoimage->width+j];
	//		image1->imageData[i*image1->widthStep+j*3+2]=stereoimage->I1_l[i*stereoimage->width+j];
	//		image2->imageData[i*image1->widthStep+j*3+0]=stereoimage->I2_l[i*stereoimage->width+j];
	//		image2->imageData[i*image1->widthStep+j*3+1]=stereoimage->I2_l[i*stereoimage->width+j];
	//		image2->imageData[i*image1->widthStep+j*3+2]=stereoimage->I2_l[i*stereoimage->width+j];
	//		/*image1_depth->imageData[i*image1_depth->widthStep+j]=stereoimage->D1[i*stereoimage->width+j];
	//		image2_depth->imageData[i*image1_depth->widthStep+j]=stereoimage->D2[i*stereoimage->width+j];*/
	//	}
	//}

	//cout<<"choose triangle:"<<tri_match[random_choose].tri_index2<<endl;

	//int tri_num=tri_match[random_choose].tri_index1;
	//CvPoint tri[] = {stereoimage->support1[stereoimage->tri1[tri_num].c1].u,stereoimage->support1[stereoimage->tri1[tri_num].c1].v,
	//					stereoimage->support1[stereoimage->tri1[tri_num].c2].u,stereoimage->support1[stereoimage->tri1[tri_num].c2].v,
	//					stereoimage->support1[stereoimage->tri1[tri_num].c3].u,stereoimage->support1[stereoimage->tri1[tri_num].c3].v};
	//CvPoint* curveArr[1]={tri};  
	//int      nCurvePts[1]={3};  
	//int      nCurves=3;  
	//int      isCurveClosed=1;  
	//int      lineWidth=1;   
	//cvPolyLine(image1,curveArr,nCurvePts,nCurves,isCurveClosed,cvScalar(255,0,0),lineWidth);
	//int tri_num1=tri_match[random_choose].tri_index2;
	//CvPoint tri1[] = {stereoimage->support2[stereoimage->tri2[tri_num1].c1].u,stereoimage->support2[stereoimage->tri2[tri_num1].c1].v,
	//					stereoimage->support2[stereoimage->tri2[tri_num1].c2].u,stereoimage->support2[stereoimage->tri2[tri_num1].c2].v,
	//					stereoimage->support2[stereoimage->tri2[tri_num1].c3].u,stereoimage->support2[stereoimage->tri2[tri_num1].c3].v};
	//CvPoint* curveArr1[1]={tri1};  
	//int      nCurvePts1[1]={3};  
	//int      nCurves1=3;  
	//int      isCurveClosed1=1;  
	//int      lineWidth1=1;   
	//cvPolyLine(image2,curveArr1,nCurvePts1,nCurves1,isCurveClosed1,cvScalar(255,0,0),lineWidth1);
	///////////////////////////////////////////
	///////////////////////////////////////////
	//R_total.inv();
	//cout<<"R_invers:"<<R_total<<endl;
	Matrix t(3,1);
	getTrans(inlier,tri_match,t,R_total);
	cout<<"t:"<<t<<endl;
	cv::Mat r_to(3,3,CV_32F);
	for(int i=0;i<3;i++)
			for(int j=0;j<3;j++)
				r_to.at<float>(i,j)=R_total.val[i][j];
	fs<<"R_total"<<r_to;
	fs.release();
}

//vector<int> getInlier_Tran()
//{
//	
//}
void PlaneEstimation::getTrans(vector<int> inlier,vector<stereoImage::tri_match> tri_match,Matrix& t,Matrix R)
{
	//Matrix t(3,1);
	int inlierNum=0;
	for(int i=0;i<30;i++)
	{
		vector<int>choose = getRandomSample_N(inlier.size(),3);
		/*stereoImage::tri_match t1 = tri_match[choose[0]];
		stereoImage::tri_match t2 = tri_match[choose[1]];
		stereoImage::tri_match t3 = tri_match[choose[2]];*/
		/*Matrix A(inlier.size(),3);
		Matrix b(inlier.size(),1);*/
		Matrix A(3,3);
		Matrix b(3,1);
		A.zero();
		for(int i=0;i<3;i++)
		{
			int active=inlier[choose[i]];
			A.val[i][0]=tri_match[active].t2a;
			A.val[i][1]=tri_match[active].t2b;
			//A.val[i][2]=tri_match[inlier[i]].t2c;
			A.val[i][2]=-1.0;
			//b.val[i][0]=tri_match[choose[i]].t1c-tri_match[choose[i]].t2c;
			double d1=abs(tri_match[active].t2c)/sqrt(tri_match[active].t2a*tri_match[active].t2a+tri_match[active].t2b*tri_match[active].t2b+1);
			double d2=abs(tri_match[active].t1c)/sqrt(tri_match[active].t1a*tri_match[active].t1a+tri_match[active].t1b*tri_match[active].t1b+1);
			b.val[i][0]=d1-d2;
		}

		/*Matrix a_at=~A*A;
		Matrix i(3,3);
		i.eye();
		i.solve(a_at);
		t=i*~A*b;*/
		if(b.solve(A))
		{
			vector<int> inlier_m = getTransInliers(inlier,tri_match,b);
			/*if((double)inlier_m.size()/(double)inlier.size() > 0.9)
			{
				t = b;
				cout<<"inlier percent:"<<(double)inlier_m.size()/(double)inlier.size()<<endl;
				break;
			}
			else
			{*/
				if(inlier_m.size() > inlierNum)
				{
					inlierNum = inlier_m.size();
					cout<<"inlier percent:"<<(double)inlierNum/(double)inlier.size()<<endl;
					t= b;
				}
			//}
		}

	}
	//cout<<"A:"<<A<<endl;

}

vector<int> PlaneEstimation::getTransInliers(vector<int> inlier,vector<stereoImage::tri_match> tri_match,Matrix b)
{
	vector<int> transInlier;
	for(int i=0;i<inlier.size();i++)
	{
		stereoImage::tri_match inlier_match = tri_match[inlier[i]];
		Matrix A(3,1);
		//Matrix b(1,1);
		//double b;
		A.val[0][0] = inlier_match.t2a;
		A.val[1][0] = inlier_match.t2b;
		A.val[2][0] = -1.0;
		double d1=abs(inlier_match.t2c)/sqrt(inlier_match.t2a*inlier_match.t2a+inlier_match.t2b*inlier_match.t2b+1);
		double d2=abs(inlier_match.t1c)/sqrt(inlier_match.t1a*inlier_match.t1a+inlier_match.t1b*inlier_match.t1b+1);
		double b =  d1-d2;
		if((~A*b).val[0][0] - b < 0.01)
			transInlier.push_back(inlier[i]);
	}
	return transInlier;
}
vector<stereoImage::tri_match> PlaneEstimation::getValidTriMatch(vector<stereoImage::tri_match> tm)
{
	Matrix a(3,1);
	Matrix b(3,1);
	vector<stereoImage::tri_match> result;
	for(int i=0;i<tm.size();i++)
	{
		a.val[0][0]=tm[i].t1a;
		a.val[1][0]=tm[i].t1b;
		//a.val[2][0]=tm[i].t1c;
		a.val[2][0]=-1.0;
		b.val[0][0]=tm[i].t2a;
		b.val[1][0]=tm[i].t2b;
		//b.val[2][0]=tm[i].t2c;
		b.val[2][0]=-1.0;

		matrix_norm(a);
		matrix_norm(b);

		if(getInnerProduct(a,b) > 0.98 && getInnerProduct(a,b)<1.0)
		{
			result.push_back(tm[i]);
		}
	}
	return result;
}
int PlaneEstimation::getRotationMatrix(stereoImage::tri_match tm,Matrix &R)
{
	Matrix a(3,1);
	Matrix b(3,1);
	a.val[0][0]  = tm.t1a;
	a.val[1][0]  = tm.t1b;
	//a.val[2][0]  = tm.t1c;
	a.val[2][0] = -1.0;
	b.val[0][0]  = tm.t2a;
	b.val[1][0]  = tm.t2b;
	//b.val[2][0]  = tm.t2c;
	b.val[2][0]=-1.0;
	matrix_norm(a);
	matrix_norm(b);
	/*cout<<"a:"<<a<<endl;
	cout<<"b:"<<b<<endl;*/
	/*if(getInnerProduct(a,b)<0.85)
		a=-a;*/
	Matrix n = Matrix::cross(a,b);
	//cout<<"n:"<<n<<endl;
	matrix_norm(n);
	//cout<<"n:"<<n<<endl;
	double cos_thita = getInnerProduct(a,b);
	double sin_thita = sqrt(1.0-cos_thita*cos_thita);
	Matrix n_anti = antisymmetricMatrix(n);
	Matrix n_tran = ~n;

	Matrix iden(3,3);
	iden.eye();
	//printf("cos_thita:%f\n",cos_thita);
	/*cout<<"iden * cos_thita:"<<n<<endl;
	cout<<"iden * cos_thita:"<<n_anti<<endl;*/
	//cout<<"iden * cos_thita:"<<n_tran<<endl;
	//罗德里格斯公式
	R = iden*cos_thita + (n*n_tran)*(1-cos_thita)+n_anti*sin_thita;
	//cout<<"det of R:"<<R.det()<<endl;
	//cout<<"R*RT:"<<endl;
	//cout<<R*~R<<endl;
	return 1;
}

Matrix PlaneEstimation::transposition(Matrix n)
{
	if(n.m!= 3 || n.n!= 1)
	{
		cout<<"error in antisymmetrix matrix"<<endl;
		exit(0);
	}
	Matrix n_tran(1,3);
	n_tran.val[0][0] = n.val[0][0];
	n_tran.val[0][1] = n.val[1][0];
	n_tran.val[0][2] = n.val[2][0];

	return n_tran;
}
Matrix PlaneEstimation::antisymmetricMatrix(Matrix n)
{
	if(n.m!= 3 || n.n!= 1)
	{
		cout<<"error in antisymmetrix matrix"<<endl;
		exit(0);
	}
	Matrix n_anti(3,3);
	n_anti.val[0][0] = 0;
	n_anti.val[0][1] = -n.val[2][0];
	n_anti.val[0][2] = n.val[1][0];
	n_anti.val[1][0] = n.val[2][0];
	n_anti.val[1][1] = 0;
	n_anti.val[1][2] = -n.val[0][0];
	n_anti.val[2][0] = -n.val[1][0];
	n_anti.val[2][1] = n.val[0][0];
	n_anti.val[2][2] = 0;

	return n_anti;
}
double PlaneEstimation::getInnerProduct(Matrix a,Matrix b)
{
	if (a.m!=3 || a.n!=1 || b.m!=3 || b.n!=1) {
		cerr << "ERROR: Cross product vectors must be of size (3x1)" << endl;
		exit(0);
	}
	double product = 0;
	product = a.val[0][0]*b.val[0][0]+a.val[1][0]*b.val[1][0]+a.val[2][0]*b.val[2][0];
	return product;
}
void PlaneEstimation::matrix_norm(Matrix& a)
{
	double norm = a.l2norm();
	a.val[0][0] = a.val[0][0]/norm;
	a.val[1][0] = a.val[1][0]/norm;
	a.val[2][0] = a.val[2][0]/norm;
}

#define RESIDUAL 0.5
vector<int> PlaneEstimation::getInlier(vector<stereoImage::tri_match> tm,Matrix R)
{
	/*Matrix A(3,1);
	Matrix B(3,1);*/
	Matrix A(1,3);
	Matrix B(1,3);
	vector<int> inlier;
	inlier.clear();
	for(int i=0;i<tm.size();i++)
	{
		A.val[0][0] = tm[i].t1a;
		A.val[0][1] = tm[i].t1b;
		//A.val[2][0] = tm[i].t1c;
		A.val[0][2]=-1.0;
		B.val[0][0] = tm[i].t2a;
		B.val[0][1] = tm[i].t2b;
		//B.val[2][0] = tm[i].t2c;
		B.val[0][2]=-1.0;
		Matrix B_hat = A*R;
		if(RESIDUAL > abs(B.val[0][0]-B_hat.val[0][0])&&
			RESIDUAL > abs(B.val[0][1]-B_hat.val[0][1])&&
			RESIDUAL > abs(B.val[0][2]-B_hat.val[0][2]))
		{
			inlier.push_back(i);
		}
	}
	return inlier;
}