#pragma once
#include <windows.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <cstdlib>
#include <vector>
#include <math.h>
#include <Eigen/Dense>
#include <algorithm>    
#include <stack>    
#include <ctime>
#include "Base.h"
#include "Helper.h"
#include "ppl.h"
#include <unordered_map>
using namespace std;
using namespace Eigen;
using namespace vcg;

typedef struct SelectNode
{
	Point3i index;
	// if front or on true;
	// else false;
	Point3<bool> flag;
	Point3<char> order;
	Point3d Point0;
	Point3d Point1;
	Point3d Point2;
	Point3d intersect1;
	Point3d intersect2;
	double insectlength;
	int faceid;
	// index of which is on the single sidevc  
};



class GirthCompute
{
public: 
	//Constructor
	GirthCompute(CMeshO& m, Point3d& inJointRoot, Point3d& inJointChild);
	//compute the girth length;
	double Compute(int intersect_Mode, Point3d upper_bound, Point3d lower_bound, int output = 0, string filename = "" , double scale = 1.0,vector<pair<Point3d, Point3d>>* copyPC = NULL);
	double Compute(Point3d upper_bound, int output = 0, string filename = "" , double scale = 1.0,vector<pair<Point3d, Point3d>>* copyPC = NULL);
	double Compute(Point3d upper_bound, int output, string filename, double scale, vector<SelectNode>& copySN);
	void Summarize(vector<Point3d> &Points, vector<Point3i> &Faces,vector<SelectNode> &SelectIndex, Point4d& PlaneParameter);
	void IntersectPlane(vector<SelectNode> &SelectIndex, Point4d& PlaneParameter, Point3d& center);
	vector<SelectNode> FindGap(const vector<SelectNode>& SelectIndex);
	void OutputIntersect(string s, int intersect_Mode, vector<SelectNode>* sI);
	void ComputeSample(int samplesize = 10);
	void ComputeSample(int samplesize, vector<double> & res, vector<vector<Point3d>>* outputbuff);
	void ComputeSampleLinearXZ(int samplesize, vector<double> & resX, vector<double> & resZ);
	bool checkifloop(vector<SelectNode>& selectIndex);
private:
	CMeshO mesh;
	vector<Point3d> Points;
	vector<Point3i> Faces;
	Point3d JointRoot;
	Point3d JointChild;
	Point3d PlaneN;
};


GirthCompute::GirthCompute(CMeshO& m, Point3d& inJointRoot, Point3d& inJointChild)
{
	vcg::tri::Append<CMeshO, CMeshO>::MeshCopy(mesh, m);
	JointRoot = inJointRoot;
	JointChild = inJointChild;
	PlaneN = (JointRoot - JointChild).Normalize();

	Points.resize(mesh.vert.size());
	Faces.resize(mesh.face.size());
	for(int i = 0; i < mesh.vert.size(); i++)
	{
		Points[i] = mesh.vert[i].P();
	}

	CVertexO * base = &mesh.vert[0];
	for(int i = 0; i < mesh.face.size(); i++)
	{
		Faces[i].X() = mesh.face[i].V(0) - base; 
		Faces[i].Y() = mesh.face[i].V(1) - base; 
		Faces[i].Z() = mesh.face[i].V(2) - base; 
	}
	vector<int> boundarybuff;
	geohelp::boundaryCheck(&mesh, boundarybuff);
	vcg::tri::UpdateColor<CMeshO>::PerVertexConstant(mesh, Color4b::Gray150);
	for(int i = 0 ; i < boundarybuff.size(); i++)
	{
		mesh.vert[boundarybuff[i]].C() = Color4b::Green;
	}
}

void GirthCompute::OutputIntersect(string s, int intersect_Mode, vector<SelectNode>* sI = NULL)
{
	vector<SelectNode> * _selectIndex = NULL;
	if(sI != NULL)
		_selectIndex = sI;

	if(_selectIndex == NULL || _selectIndex->size() == 0)
		return;

	vector<CVertexO> outputbuff(_selectIndex->size());
	for(int i = 0 ; i < _selectIndex->size(); i++)
	{
		outputbuff[i].P() = _selectIndex->at(i).intersect1;
		outputbuff[i].N() = Point3d(0,1,0);
		if(intersect_Mode == 1)
			outputbuff[i].C() = Color4b::Red;
		else if(intersect_Mode == 2)
			outputbuff[i].C() = Color4b::Green;
		else if(intersect_Mode == 3)
			outputbuff[i].C() = Color4b::Yellow;
		else if(intersect_Mode == 4)
			outputbuff[i].C() = Color4b::Blue;
		else
			outputbuff[i].C() = Color4b::Blue;
	}

	Log::LogPC_CVertexO(s.c_str(), &outputbuff);
}


void GirthCompute::ComputeSample(int samplesize)
{
	double bestpos = 0;
	double k = 0;
	int i = 0;
	for(;i < samplesize; i++, k += 1.0 / samplesize)
	{
		Point3d PassPoint = (JointChild - JointRoot) * k + JointRoot;
		Point4d PlaneParam(PlaneN.X(), PlaneN.Y(), PlaneN.Z(), 0);
		PlaneParam.W() = -1 * (PassPoint.X() * PlaneN.X() + PassPoint.Y() * PlaneN.Y() + PassPoint.Z() * PlaneN.Z());

		vector<SelectNode> selectIndex;
		Summarize(this->Points, this->Faces, selectIndex, PlaneParam);
		IntersectPlane(selectIndex, PlaneParam, PassPoint);
		OutputIntersect("Mesh/Samples/" + to_string(i) + ".ply", 5, &selectIndex);
	}

}

bool GirthCompute::checkifloop(vector<SelectNode>& selectIndex)
{
	for(auto i : selectIndex)
	{
		if(mesh.face[i.faceid].V(0)->C().Equal((Color4b)Color4b::Green))
			return false;
		if(mesh.face[i.faceid].V(1)->C().Equal((Color4b)Color4b::Green))
			return false;
		if(mesh.face[i.faceid].V(2)->C().Equal((Color4b)Color4b::Green))
			return false;
	}
	return true;
}
void GirthCompute::ComputeSample(int samplesize, vector<double> & res, vector<vector<Point3d>>* outputbuff = NULL)
{
	res.resize(samplesize, 0);
	double k = 0;
	int i = 0;
	if(outputbuff)
		outputbuff->resize(samplesize);
	for(;i < samplesize; i++, k += 1.0 / samplesize)
	{
		Point3d PassPoint = (JointChild - JointRoot) * k + JointRoot;
		Point4d PlaneParam(PlaneN.X(), PlaneN.Y(), PlaneN.Z(), 0);
		PlaneParam.W() = -1 * (PassPoint.X() * PlaneN.X() + PassPoint.Y() * PlaneN.Y() + PassPoint.Z() * PlaneN.Z());

		vector<SelectNode> selectIndex;
		Summarize(this->Points, this->Faces, selectIndex, PlaneParam);
		IntersectPlane(selectIndex, PlaneParam, PassPoint);
		double girth = 0;
		if(!checkifloop(selectIndex))
		{
			res[i] = 0;
			continue;
		}
		
		for(int j = 0; j < selectIndex.size(); j++)
		{
			girth += selectIndex[j].insectlength;
		}
		res[i] = girth;

		for(int j = 0; j < selectIndex.size(); j++)
		{
			outputbuff->at(i).push_back(selectIndex[j].intersect1);
		}
	}
}

void GirthCompute::ComputeSampleLinearXZ(int samplesize, vector<double> & resX, vector<double> & resZ)
{
	resX.resize(samplesize, 0);
	resZ.resize(samplesize, 0);
	double k = 0;
	int i = 0;
	for(;i < samplesize; i++, k += 1.0 / samplesize)
	{
		Point3d PassPoint = (JointChild - JointRoot) * k + JointRoot;
		Point4d PlaneParam(PlaneN.X(), PlaneN.Y(), PlaneN.Z(), 0);
		PlaneParam.W() = -1 * (PassPoint.X() * PlaneN.X() + PassPoint.Y() * PlaneN.Y() + PassPoint.Z() * PlaneN.Z());

		vector<SelectNode> selectIndex;
		Summarize(this->Points, this->Faces, selectIndex, PlaneParam);
		IntersectPlane(selectIndex, PlaneParam, PassPoint);
		if(!checkifloop(selectIndex))
		{
			resX[i] = 0;
			resZ[i] = 0;
			continue;
		}
		
		double X_max = 0, X_min = 1e4, Z_max = 0, Z_min = 1e4;
		for(int i = 0; i < selectIndex.size(); i++)
		{
			double currX = selectIndex[i].intersect1.X();
			double currZ = selectIndex[i].intersect1.Z();
			X_max = max(currX, X_max);
			X_min = min(currX, X_min);
			Z_max = max(currZ, Z_max);
			Z_min = min(currZ, Z_min);
		}
		resX[i] = fabs(X_max - X_min) > 1e4? 0 : (X_max - X_min);
		resZ[i] = fabs(Z_max - Z_min) > 1e4? 0 : (Z_max - Z_min);;
	}
}

double GirthCompute::Compute(Point3d upper_bound, int output, string filename, double scale, vector<pair<Point3d, Point3d>>* copyPC)
{
	//default : Mode 4: GivenPoint
	vector<SelectNode> Intersect;
	vector<SelectNode> * _selectIndex = &Intersect;
	double upper_rate = (upper_bound.Y() - this->JointChild.Y()) / (this->JointRoot.Y() - this->JointChild.Y());

	_selectIndex->clear();
	Point3d PassPoint = (JointRoot - JointChild) * upper_rate + JointChild;
	Point4d PlaneParam(PlaneN.X(), PlaneN.Y(), PlaneN.Z(), 0);
	PlaneParam.W() = -1 * (PassPoint.X() * PlaneN.X() + PassPoint.Y() * PlaneN.Y() + PassPoint.Z() * PlaneN.Z());

	Summarize(this->Points, this->Faces, *_selectIndex, PlaneParam);
	IntersectPlane(*_selectIndex, PlaneParam, PassPoint);

	double sum = 0;
	for(auto& i : *_selectIndex)
	{
		sum += i.insectlength;
	}


	if(output != 0)
		OutputIntersect(filename, 4, _selectIndex);
	if(copyPC != NULL)
	{
		copyPC->resize(_selectIndex->size());
		for(int i = 0 ; i < _selectIndex->size(); i++)
		{
			copyPC->at(i).first = _selectIndex->at(i).intersect1;
			copyPC->at(i).second = _selectIndex->at(i).intersect2;
		}
	}
	//cout<<"Linear Length  = "<< sum * scale << endl << endl;
	return sum * scale;
}

double GirthCompute::Compute(Point3d upper_bound, int output, string filename, double scale, vector<SelectNode>& copySN)
{
	//default : Mode 4: GivenPoint
	vector<SelectNode> Intersect;
	vector<SelectNode> * _selectIndex = &Intersect;
	double upper_rate = (upper_bound.Y() - this->JointChild.Y()) / (this->JointRoot.Y() - this->JointChild.Y());

	_selectIndex->clear();
	Point3d PassPoint = (JointRoot - JointChild) * upper_rate + JointChild;
	Point4d PlaneParam(PlaneN.X(), PlaneN.Y(), PlaneN.Z(), 0);
	PlaneParam.W() = -1 * (PassPoint.X() * PlaneN.X() + PassPoint.Y() * PlaneN.Y() + PassPoint.Z() * PlaneN.Z());

	Summarize(this->Points, this->Faces, *_selectIndex, PlaneParam);
	IntersectPlane(*_selectIndex, PlaneParam, PassPoint);

	double sum = 0;
	for(auto& i : *_selectIndex)
	{
		sum += i.insectlength;
	}


	if(output != 0)
		OutputIntersect(filename, 4, _selectIndex);
	copySN.resize(_selectIndex->size());
	for(int i =0 ; i < _selectIndex->size(); i++)
		copySN[i] = _selectIndex->at(i);
	
	cout<<"Linear Length  = "<< sum * scale << endl << endl;
	return sum * scale;
}

double GirthCompute::Compute(int intersect_Mode, Point3d upper_bound, Point3d lower_bound, int output, string filename, double scale, vector<pair<Point3d, Point3d>>* copyPC)
{
	vector<SelectNode> Intersect;
	vector<SelectNode> * _selectIndex = &Intersect;
	double upper_rate = 0, lower_rate = 0;
	if( this->JointChild.Y() != this->JointRoot.Y())
	{
		upper_rate = (upper_bound.Y() - this->JointChild.Y()) / (this->JointRoot.Y() - this->JointChild.Y());
		lower_rate = intersect_Mode == 4 ? upper_rate: (lower_bound.Y() - this->JointChild.Y()) / (this->JointRoot.Y() - this->JointChild.Y());
	}
	else if(this->JointChild.X() != this->JointRoot.X())
	{
		upper_rate = (upper_bound.X() - this->JointChild.X()) / (this->JointRoot.X() - this->JointChild.X());
		lower_rate = intersect_Mode == 4 ? upper_rate: (lower_bound.X() - this->JointChild.X()) / (this->JointRoot.X() - this->JointChild.X());
	}
	if(intersect_Mode == 1)
	{
		upper_rate = (upper_rate + lower_rate) * .5;
		lower_rate = upper_rate;
	}


	//1: mid 2：max 3: min 4: given pt
	double allsum = intersect_Mode == 3 ? FLT_MAX : 0.0;
	double prop = upper_rate;
	double bestpos = prop;
	double factor = 0.01;

	while(prop <= lower_rate)
	{
		_selectIndex->clear();
		Point3d PassPoint = (JointRoot - JointChild) * prop + JointChild;
		Point4d PlaneParam(PlaneN.X(), PlaneN.Y(), PlaneN.Z(), 0);
		PlaneParam.W() = -1 * (PassPoint.X() * PlaneN.X() + PassPoint.Y() * PlaneN.Y() + PassPoint.Z() * PlaneN.Z());

		Summarize(this->Points, this->Faces, *_selectIndex, PlaneParam);
		IntersectPlane(*_selectIndex, PlaneParam, PassPoint);

		double sum = 0;
		for(auto& i : *_selectIndex)
		{
			sum += i.insectlength;
		}
		if(intersect_Mode == 1 || intersect_Mode == 2 || intersect_Mode == 4)
		{
			if(sum > allsum)
			{
				bestpos = prop;
				allsum = sum;
			}
		}
		else if(intersect_Mode == 3)
		{
			if(sum < allsum)
			{
				bestpos = prop;
				allsum = sum;
			}
		}
		prop += factor;
	}

	if(intersect_Mode == 2 || intersect_Mode == 3)
	{
		_selectIndex->clear();
		Point3d PassPoint = (JointRoot - JointChild) * bestpos + JointChild;
		Point4d PlaneParam(PlaneN.X(), PlaneN.Y(), PlaneN.Z(), 0);
		PlaneParam.W() = -1 * (PassPoint.X() * PlaneN.X() + PassPoint.Y() * PlaneN.Y() + PassPoint.Z() * PlaneN.Z());

		Summarize(this->Points, this->Faces, *_selectIndex, PlaneParam);
		IntersectPlane(*_selectIndex, PlaneParam, PassPoint);
	}
	if(output != 0)
	{
		OutputIntersect(filename, intersect_Mode, _selectIndex);
	}
	if(copyPC != NULL)
	{
		copyPC->resize(_selectIndex->size());
		for(int i = 0 ; i < _selectIndex->size(); i++)
		{
			copyPC->at(i).first = _selectIndex->at(i).intersect1;
			copyPC->at(i).second = _selectIndex->at(i).intersect2;
		}
	}

	cout<<"intersect mode = "<< intersect_Mode<<endl;
	cout<<"Linear Length  = "<< allsum * scale << endl << endl;
	return allsum * scale;
}




void GirthCompute::IntersectPlane(vector<SelectNode> &SelectIndex, Point4d& PlaneParameter, Point3d& center)
{
	for(int i=0; i<SelectIndex.size(); i++)
	{
		int single;
		if(SelectIndex[i].flag.X() == SelectIndex[i].flag.Y())
			single = 2; //z
		else if(SelectIndex[i].flag.X() == SelectIndex[i].flag.Z())
			single = 1; //y
		else 
			single = 0; //x
		Point3d unique, dule1, dule2;
		switch (single)
		{
		case 0:
			unique=SelectIndex[i].Point0 - center;
			dule1 =SelectIndex[i].Point1 - center;
			dule2 =SelectIndex[i].Point2 - center;
			SelectIndex[i].order = Point3<char>(0,1,2);
			break;
		case 1:
			unique=SelectIndex[i].Point1 - center;
			dule1 =SelectIndex[i].Point0 - center;
			dule2 =SelectIndex[i].Point2 - center;
			SelectIndex[i].order = Point3<char>(1,0,2);
			break;
		case 2:
			unique=SelectIndex[i].Point2 - center;
			dule1 =SelectIndex[i].Point0 - center;
			dule2 =SelectIndex[i].Point1 - center;
			SelectIndex[i].order = Point3<char>(2,0,1);
			break;
		default:
			break;
		}

		double A=PlaneParameter.X();
		double B=PlaneParameter.Y();
		double C=PlaneParameter.Z();
		double t1= (-A*unique.X()-B*unique.Y()-C*unique.Z())/(A*(dule1.X()-unique.X())+B*(dule1.Y()-unique.Y())+C*(dule1.Z()-unique.Z()));
		double t2= (-A*unique.X()-B*unique.Y()-C*unique.Z())/(A*(dule2.X()-unique.X())+B*(dule2.Y()-unique.Y())+C*(dule2.Z()-unique.Z()));
		Point3d insect1, insect2;
		insect1.X() = unique.X()+t1*(dule1.X()-unique.X());
		insect1.Y() = unique.Y()+t1*(dule1.Y()-unique.Y()), 
			insect1.Z() = unique.Z()+t1*(dule1.Z()-unique.Z());
		insect2.X() = unique.X()+t2*(dule2.X()-unique.X());
		insect2.Y() = unique.Y()+t2*(dule2.Y()-unique.Y());
		insect2.Z() = unique.Z()+t2*(dule2.Z()-unique.Z());

		SelectIndex[i].intersect1=insect1 + center;
		SelectIndex[i].intersect2=insect2 + center;
		SelectIndex[i].insectlength= Point3d(insect1 - insect2).Norm();
	}
}


inline bool is_front_plane(Point3d& p, Point4d& PlaneParameter)
{
	double d = PlaneParameter.X() *p.X() + PlaneParameter.Y() * p.Y() + PlaneParameter.Z() * p.Z() + PlaneParameter.W();
	if(d >= 0) return true;
	else return false;
}

void GirthCompute::Summarize(vector<Point3d> &Points, vector<Point3i> &Faces,vector<SelectNode> &SelectIndex, Point4d& PlaneParameter)
{
	SelectIndex.resize(0);
	SelectIndex.reserve(1024);
	for(int i=0; i<Faces.size(); i++)
	{
		Point3d P[3];
		bool B[3];

		P[0]=Points[Faces[i].X()];
		P[1]=Points[Faces[i].Y()];
		P[2]=Points[Faces[i].Z()];

		B[0]=is_front_plane(P[0] , PlaneParameter);
		B[1]=is_front_plane(P[1] , PlaneParameter);
		B[2]=is_front_plane(P[2] , PlaneParameter);

		bool And=B[0] && B[1] && B[2];
		bool Or =B[0] || B[1] || B[2];
		if(And!=Or)
		{
			SelectNode S;
			S.index=Faces[i];
			S.flag.X() = B[0];
			S.flag.Y() = B[1];
			S.flag.Z() = B[2];
			S.Point0 = P[0];
			S.Point1 = P[1];
			S.Point2 = P[2];
			S.intersect1.SetZero();
			S.intersect2.SetZero();
			S.insectlength = 0;
			S.faceid = i;
			SelectIndex.push_back(S);
		}
	}
}


vector<SelectNode> GirthCompute::FindGap(const vector<SelectNode>& SelectIndex)
{
	vector<SelectNode> breakpoint;
	vector<SelectNode> memo;
	memo.reserve(SelectIndex.size());
	unordered_map<int, int> facetoselect;
	int n = SelectIndex.size();
	for(int i = 0; i < n; i++)
	{
		facetoselect[SelectIndex[i].faceid] = i;
	}
	SelectNode currSelect = SelectIndex[0];
	int i = 1; 

	Point2i edgeShare;
	Point2i edgeShare_inv;
	if(currSelect.order.X() == 0)
	{
		edgeShare = Point2i(SelectIndex[0].index.X(), SelectIndex[0].index.Y());
		edgeShare_inv = Point2i(SelectIndex[0].index.X(), SelectIndex[0].index.Z());
	}
	else if(currSelect.order.X() == 1)
	{
		edgeShare = Point2i(SelectIndex[0].index.Y(), SelectIndex[0].index.X());
		edgeShare_inv = Point2i(SelectIndex[0].index.Y(), SelectIndex[0].index.Z());
	}
	else 
	{
		edgeShare = Point2i(SelectIndex[0].index.Z(), SelectIndex[0].index.X());
		edgeShare_inv = Point2i(SelectIndex[0].index.Z(), SelectIndex[0].index.Y());
	}

	vector<int> gapidx;

	CFaceO* Pfirst = mesh.face.data();
	CVertexO* Vfirst = mesh.vert.data();
	memo.push_back(currSelect);
	vcg::tri::UpdateTopology<CMeshO>::VertexFace(mesh);

	while(i <= n)
	{
		int faceid = currSelect.faceid;
		mesh.face[faceid].SetS();
		CVertexO* v;
		if(currSelect.order.X() == 0)
			v = mesh.vert.data() + ((int)currSelect.index.X());
		else if(currSelect.order.X() == 1)
			v = mesh.vert.data() + ((int)currSelect.index.Y());
		else
			v = mesh.vert.data() + ((int)currSelect.index.Z());

		vcg::face::VFIterator<CFaceO> vfi(v);
		int faceid_neighbor = -1;
		for(int k = 0; !vfi.End(); ++vfi, k++)
		{
			CFaceO* f = vfi.F();
			int v0 = f->V(0) - Vfirst;
			int v1 = f->V(1) - Vfirst;
			int v2 = f->V(2) - Vfirst;
			if((v0 == edgeShare.X() || v1 == edgeShare.X() || v2 == edgeShare.X()) && (v0 == edgeShare.Y() || v1 == edgeShare.Y() || v2 == edgeShare.Y()))
			{
				if(!f->IsS())
				{
					faceid_neighbor = f - Pfirst;
					break;
				}
			}
		}
		if(faceid_neighbor == -1)
		{
			gapidx.push_back(i - 1);
			if(gapidx.size() < 2)
			{
				edgeShare = edgeShare_inv;
				currSelect = memo.front();
				continue;
			}
			else
			{
				break;
			}

		}
		int newselectidx = facetoselect[faceid_neighbor];
		memo.push_back(SelectIndex[newselectidx]);
		currSelect = memo.back();

		Point2i es;
		if(currSelect.order.X() == 0)
		{
			es.X() = currSelect.index.X();
			if(currSelect.index.Y() == edgeShare.X() || currSelect.index.Y() == edgeShare.Y())
			{
				es.Y() = currSelect.index.Z();
			}
			else
			{
				es.Y() = currSelect.index.Y();
			}
		}
		else if(currSelect.order.X() == 1)
		{
			es.X() = currSelect.index.Y();
			if(currSelect.index.X() == edgeShare.X() || currSelect.index.X() == edgeShare.Y())
			{
				es.Y() = currSelect.index.Z();
			}
			else
			{
				es.Y() = currSelect.index.X();
			}
		}
		else if(currSelect.order.X() == 2)
		{
			es.X() = currSelect.index.Z();
			if(currSelect.index.X() == edgeShare.X() || currSelect.index.X() == edgeShare.Y())
			{
				es.Y() = currSelect.index.Y();
			}
			else
			{
				es.Y() = currSelect.index.X();
			}
		}
		edgeShare = es;
		i++;
	}

	vector<SelectNode> res;
	res.push_back(memo.at(gapidx[0]));
	res.push_back(memo.back());
	return res;
}
