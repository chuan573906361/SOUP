#include "stdafx.h"
#ifndef __DELAUNAY_H__
#define __DELAUNAY_H__
#define MAX_VERTEX_NUM 4092
#define SINGLE
#ifdef SINGLE
#define DREAL float
#else
#define DREAL double
#endif


// TYPES //////////////////////////////////////////////////

typedef struct VERTEX2D_TYP
{
	DREAL x;
	DREAL y;

} VERTEX2D, *VERTEX2D_PTR;

typedef struct EDGE_TYP
{
	VERTEX2D v1;
	VERTEX2D v2;

} EDGE, *EDGE_PTR;

typedef struct TRIANGLE_TYP
{
	int i1; // vertex index
	int i2; 
	int i3; 

	TRIANGLE_TYP* pNext;
	TRIANGLE_TYP* pPrev;

} TRIANGLE, *TRIANGLE_PTR;

typedef struct MESH_TYP
{
	int vertex_num;
	int triangle_num;

	VERTEX2D_PTR pVerArr; // point to outer vertices arrary
	TRIANGLE_PTR pTriArr; // point to outer triangles arrary

} MESH, *MESH_PTR;



// PROTOTYPES ///////////////////////////////////////////
// Delaunay triangulation functions
void InitMesh(MESH_PTR pMesh, int ver_num);
void UnInitMesh(MESH_PTR pMesh);

void AddBoundingBox(MESH_PTR pMesh);
void RemoveBoundingBox(MESH_PTR pMesh);;
void IncrementalDelaunay(MESH_PTR pMesh);

void Insert(MESH_PTR pMesh, int ver_index);
bool FlipTest(MESH_PTR pMesh, TRIANGLE_PTR pTestTri);

DREAL InCircle(VERTEX2D_PTR pa, VERTEX2D_PTR pb, VERTEX2D_PTR pp, VERTEX2D_PTR  pd);
DREAL InTriangle(MESH_PTR pMesh, VERTEX2D_PTR pVer, TRIANGLE_PTR pTri);

void InsertInTriangle(MESH_PTR pMesh, TRIANGLE_PTR pTargetTri, int ver_index);
void InsertOnEdge(MESH_PTR pMesh, TRIANGLE_PTR pTargetTri, int ver_index);

// Helper functions
void RemoveTriangleNode(MESH_PTR pMesh, TRIANGLE_PTR pTri);
TRIANGLE_PTR AddTriangleNode(MESH_PTR pMesh, TRIANGLE_PTR pPrevTri, int i1, int i2, int i3);

// I/O functions
void Input(char* pFile, MESH_PTR pMesh);
void Output(char* pFile, MESH_PTR pMesh);
#endif