#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#define WIN32_LEAN_AND_MEAN		// Exclude rarely-used stuff from Windows headers

#include <stdio.h>

#ifndef SAFE_DELETE
#define SAFE_DELETE(p) { if (p) { delete (p); (p)=NULL; } }
#endif

#ifndef SAFE_DELETE_ARRAY
#define SAFE_DELETE_ARRAY(p) { if (p) { delete[] (p); (p)=NULL; } }
#endif

#define PI 3.14159265
#define DEBUG 1
#define SCALE 1000

#pragma once
#include <Eigen/Dense>
#include "ppl.h"
#define NUM_FACE 8
using namespace Eigen;
typedef Matrix<double, 3, 3, RowMajor> Mat3;
typedef Matrix<double, 4, 4, RowMajor> Mat4;
typedef MatrixXd MatX;

typedef Vector3d Vet3;
typedef Vector4d Vet4;
typedef VectorXd VetX;

typedef unsigned int UINT;
typedef unsigned char BYTE;

struct BaryCoord{
	int idx;	//mesh idx
	float u;	//BaryCoordinate u;
	float v;	//BaryCoordinate v;	// u + v + w = 1
};

struct GlobalCorr{
	BaryCoord  Corr_T;	//correspondence Coordinate in target surface
	int VexId_S;		//Vertex id on source surface
	int MeshId_S;		//Source mesh Id
	int MeshId_T;		//Target mesh Id
};

struct Coo{
	int row;
	int col;
	double val;
};

enum JointsName
{
	//trunk	 neck, shoulder, elbow, wrist, thigh, knee, ankle (repeat) shoulder, elbow, wrist, thigh, knee, ankle
	center_trunk, center_abodmen, center_waist, center_neck, left_arm_1, left_arm_2, left_arm_3, left_leg_1, left_leg_2, left_leg_3, right_arm_1, right_arm_2, right_arm_3, right_leg_1, right_leg_2, right_leg_3
};
enum SegmentName
{
	segment_trunk, segment_armleft, segment_armright, segment_legleft, segment_legright, segment_android, segment_gynoid, segment_head
};
#define DEBUGPATH "D:\\BodyFatFeatureExtraction\\Data Debug\\"
#define Debug 0

