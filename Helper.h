#pragma once
#include "base.h"

#include <wrap/ply/plylib.h>
#include <wrap/ply/plystuff.h>
#include <wrap/io_trimesh/import.h>
#include <wrap/io_trimesh/export_ply.h>
#include <wrap/ply/plylib.cpp>
#include <unordered_map>
#include <vcg/complex/algorithms/smooth.h>
#include <set>
#include "Nanoknn.h"
using namespace Eigen;
using namespace vcg;


class Log
{
public:
	inline static void LogKTXT(const char* filename, vector<double>& QTable, int w, int h)
	{
		ofstream myfile;
		myfile.open (filename);
		for(int y = 0; y < h; y++)
		{
			for(int x = 0; x < w; x++)
			{
				myfile<< QTable[y * w + x]<<"\t";
			}
			myfile<<"\n";
		}
		myfile.close();
		cout<<"Log file "<< *filename<<" Done!"; 
	}

	inline static void LoadMesh(const char* filename, CMeshO & m)
	{
		int err=vcg::tri::io::Importer<vcg::CMeshO>::Open(m, filename);
		if(err)	{
			std::cerr << "Unable to open mesh " << filename << " : " << vcg::tri::io::Importer<vcg::CMeshO>::ErrorMsg(err) << std::endl;
			exit(-1);
		}

		tri::UpdateNormal<CMeshO>::PerVertex(m);
		tri::UpdateNormal<CMeshO>::NormalizePerVertex(m);

		printf( "%s------> has %i vert and %i faces\n", filename , m.VN(),  m.FN());
	}



	static void LogMesh_CMeshO(CMeshO * m, const char * filename, bool ifNormal = true, bool ifColor = true, bool if_binary = true)
	{
		vcg::tri::UpdateNormal<CMeshO>::NormalizePerVertex(*m);
		if(ifNormal && ifColor && if_binary)
			tri::io::ExporterPLY<CMeshO>::Save(*m, filename, vcg::tri::io::Mask::IOM_VERTNORMAL+vcg::tri::io::Mask::IOM_VERTCOLOR, true);
		else if(ifColor && if_binary)
			tri::io::ExporterPLY<CMeshO>::Save(*m, filename, vcg::tri::io::Mask::IOM_VERTCOLOR, true);
		else if(ifNormal && if_binary)
			tri::io::ExporterPLY<CMeshO>::Save(*m, filename, vcg::tri::io::Mask::IOM_VERTNORMAL, true);
		else if(ifNormal && ifColor)
			tri::io::ExporterPLY<CMeshO>::Save(*m, filename, vcg::tri::io::Mask::IOM_VERTNORMAL+vcg::tri::io::Mask::IOM_VERTCOLOR, false);
		else if(ifNormal)
			tri::io::ExporterPLY<CMeshO>::Save(*m, filename, vcg::tri::io::Mask::IOM_VERTNORMAL, false);
		else if(if_binary)
			tri::io::ExporterPLY<CMeshO>::Save(*m, filename, true);
		else if(ifColor)
			tri::io::ExporterPLY<CMeshO>::Save(*m, filename, vcg::tri::io::Mask::IOM_VERTCOLOR, false);
		if(!ifNormal&&!ifColor&&!if_binary)
			tri::io::ExporterPLY<CMeshO>::Save(*m, filename, false);

		cout<<"Log PLY Model "<<filename<<" Success!"<<endl;
	}
	static void LogMesh_CMeshO_Q(CMeshO * m, const char * filename)
	{
		//vcg::tri::UpdateNormal<CMeshO>::NormalizePerVertex(*m);
		tri::io::ExporterPLY<CMeshO>::Save(*m, filename, vcg::tri::io::Mask::IOM_VERTNORMAL+vcg::tri::io::Mask::IOM_VERTCOLOR+vcg::tri::io::Mask::IOM_VERTQUALITY, false);
		cout<<"Log PLY Model "<<filename<<" Success!"<<endl;
	}

	static void LogMesh_CMeshO_SCALE(CMeshO * m, const char * filename, bool ifNormal = false, bool ifColor = false, double scale = 1)
	{
		vcg::tri::UpdatePosition<CMeshO>::Scale(*m, Point3d(scale, scale, scale));
		vcg::tri::UpdateNormal<CMeshO>::NormalizePerVertex(*m);
		tri::io::ExporterPLY<CMeshO>::Save(*m, filename, vcg::tri::io::Mask::IOM_VERTNORMAL+vcg::tri::io::Mask::IOM_VERTCOLOR, true);
		cout<<"Log PLY Model "<<filename<<" Success!"<<endl;
	}


	static void LogPC_CVertexO(const char* filename, vector<CVertexO>* bv)
	{
		FILE *fc = fopen(filename, "w+");		
		fprintf(fc,"ply\n");
		fprintf(fc,"format ascii 1.0\n");
		fprintf(fc,"comment : created from Kinect depth image\n");
		fprintf(fc,"element vertex %d\n", bv->size());
		fprintf(fc,"property float x\n");
		fprintf(fc,"property float y\n");
		fprintf(fc,"property float z\n");
		fprintf(fc,"property float nx\n");
		fprintf(fc,"property float ny\n");
		fprintf(fc,"property float nz\n");
		fprintf(fc,"property uchar red\n");
		fprintf(fc,"property uchar green\n");
		fprintf(fc,"property uchar blue\n");

		fprintf(fc,"end_header\n");

		vector<CVertexO>::iterator _iter1 = bv->begin();
		for( ; _iter1 != bv->end() ; _iter1 ++)
		{ 
			if(_iter1->P().Norm()< 100000)
			{
				fprintf( fc, "%3.4f %3.4f %3.4f ", (float)_iter1->P().X() , (float)_iter1->P().Y(), (float)_iter1->P().Z());
				fprintf( fc, "%3.4f %3.4f %3.4f ", (float)_iter1->N().X() , (float)_iter1->N().Y(), (float)_iter1->N().Z());
				fprintf( fc, "%d %d %d\n", (int)_iter1->C().X() , (int)_iter1->C().Y()  , (int)_iter1->C().Z());
			}
			else
			{
				fprintf( fc, "%3.4f %3.4f %3.4f ", (float)0 , (float)0 , (float)0);
				fprintf( fc, "%3.4f %3.4f %3.4f ", (float)0 , (float)0, (float)1);
				fprintf( fc, "%d %d %d\n",0, 0, 0);
			}
		}

		fclose (fc);
		cout<<"Log PLY Model "<<filename<<" Success!"<<endl;
	}
	static void LogPC_Point3d( vector<vector<Point3d>>& bv, const char* filename)
	{
		int cnt = 0;
		for(int j = 0; j < bv.size(); j++)
		{
			for(int i = 0; i < bv[j].size(); i++)
			{
				cnt++;
			}
		}
		FILE *fc = fopen(filename, "w+");		
		fprintf(fc,"ply\n");
		fprintf(fc,"format ascii 1.0\n");
		fprintf(fc,"comment : created from Kinect depth image\n");
		fprintf(fc,"element vertex %d\n", cnt);
		fprintf(fc,"property float x\n");
		fprintf(fc,"property float y\n");
		fprintf(fc,"property float z\n");
		fprintf(fc,"property float nx\n");
		fprintf(fc,"property float ny\n");
		fprintf(fc,"property float nz\n");
		fprintf(fc,"property uchar red\n");
		fprintf(fc,"property uchar green\n");
		fprintf(fc,"property uchar blue\n");

		fprintf(fc,"end_header\n");
		for(int j = 0; j < bv.size(); j++)
		{
			for(int i = 0; i < bv[j].size(); i++)
			{
				fprintf( fc, "%3.4f %3.4f %3.4f ", bv[j][i].X(), bv[j][i].Y(), bv[j][i].Z());
				fprintf( fc, "%3.4f %3.4f %3.4f ", 0,1,0);
				fprintf( fc, "%d %d %d\n", 255, 255, 255);
			}
		}
		fclose (fc);
		cout<<"Log PLY Model "<<filename<<" Success!"<<endl;
	}
	static void LogPC_Point3d( vector<Point3d>& bv, const char* filename)
	{
		FILE *fc = fopen(filename, "w+");		
		fprintf(fc,"ply\n");
		fprintf(fc,"format ascii 1.0\n");
		fprintf(fc,"comment : created from Kinect depth image\n");
		fprintf(fc,"element vertex %d\n", bv.size());
		fprintf(fc,"property float x\n");
		fprintf(fc,"property float y\n");
		fprintf(fc,"property float z\n");
		fprintf(fc,"property float nx\n");
		fprintf(fc,"property float ny\n");
		fprintf(fc,"property float nz\n");
		fprintf(fc,"property uchar red\n");
		fprintf(fc,"property uchar green\n");
		fprintf(fc,"property uchar blue\n");

		fprintf(fc,"end_header\n");
		for(int i = 0; i < bv.size(); i++)
		{
			fprintf( fc, "%3.4f %3.4f %3.4f ", bv[i].X(), bv[i].Y(), bv[i].Z());
			fprintf( fc, "%3.4f %3.4f %3.4f ", 0,1,0);
			fprintf( fc, "%d %d %d\n", 255, 255, 255);
		}
		fclose (fc);
		cout<<"Log PLY Model "<<filename<<" Success!"<<endl;
	}
};

class geohelp
{
public:
	static Matrix44d Points_rotate(Point3d target_vector, Point3d  source_vector)
	{
		int isflip = false;
		Matrix<double, 4, 4, RowMajor> res;
		res.setIdentity();
		Matrix44d result;
		result.SetIdentity();
		if(target_vector == source_vector)	return result;
		Point3d B = target_vector.Normalize();
		Point3d A = source_vector.Normalize();
		Vector3d xB(B.X(), B.Y(), B.Z());
		Vector3d xA(A.X(), A.Y(), A.Z());
		Vector3d x = xA.cross(xB);
		float s = x.norm();
		float cc = xA.dot(xB);
		Matrix<double, 3, 3, RowMajor> mA;
		mA.setZero();
		mA(0,1) = -x.z();
		mA(0,2) =  x.y();
		mA(1,0) =  x.z();
		mA(1,2) = -x.x();
		mA(2,0) = -x.y();
		mA(2,1) =  x.x();
		Matrix<double, 3, 3, RowMajor> R;
		R.setIdentity();
		R.block<3,3>(0,0) += mA + mA * mA * 1 / (1 + cc);
		res.block<3,3>(0,0) = R;
		result.FromEigenMatrix(res);
		return result;
	}

	static void boundaryCheckEdge(CMeshO& m, vector<pair<int,int>>& boundarylinks)
	{
		vcg::tri::UpdateFlags<CMeshO>::Clear(m);
		vcg::tri::UpdateTopology<CMeshO>::FaceFace(m);
		vcg::tri::UpdateFlags<CMeshO>::VertexClearV(m);
		boundarylinks.clear();
		boundarylinks.reserve(5000);
		CVertexO* first = m.vert.data();
		for(int k = 0; k < m.face.size(); k++)
		{
			if(face::IsBorder(m.face[k], 0))
				boundarylinks.emplace_back(m.face[k].V(0) - first, m.face[k].V(1) - first);
			if(face::IsBorder(m.face[k], 1))
				boundarylinks.emplace_back(m.face[k].V(1) - first, m.face[k].V(2) - first);
			if(face::IsBorder(m.face[k], 2))
				boundarylinks.emplace_back(m.face[k].V(2) - first, m.face[k].V(0) - first);
		}
	}
	static void boundaryCheck( CMeshO * m, vector<int> & boundary_idx)
	{
		vcg::tri::UpdateFlags<CMeshO>::Clear(*m);
		vcg::tri::UpdateTopology<CMeshO>::FaceFace(*m);
		vcg::tri::UpdateFlags<CMeshO>::VertexClearV(*m);
		boundary_idx.clear();
		boundary_idx.reserve(5000);
		CVertexO* first = m->vert.data();

		for(int k = 0 ; k < (*m).FN(); k++)
		{
			int a[3] = {0};
			if(face::IsBorder((*m).face[k],0)) //edge 0-1
			{
				if(!(*m).face[k].V(0)->IsV())
				{
					a[0] = 1;
					(*m).face[k].V(0)->SetV();
				}
				if(!(*m).face[k].V(1)->IsV())
				{
					a[1] = 1;
					(*m).face[k].V(1)->SetV();
				}
			}
			if(face::IsBorder((*m).face[k],1)) //edge 1-2
			{
				if(!(*m).face[k].V(2)->IsV())
				{
					a[2] = 1;
					(*m).face[k].V(2)->SetV();
				}
				if(!(*m).face[k].V(1)->IsV())
				{
					a[1] = 1;
					(*m).face[k].V(1)->SetV();
				}
			}
			if(face::IsBorder((*m).face[k],2)) //edge 2-0
			{
				if(!(*m).face[k].V(2)->IsV())
				{
					a[2] = 1;
					(*m).face[k].V(2)->SetV();
				}
				if(!(*m).face[k].V(0)->IsV())
				{
					a[0] = 1;
					(*m).face[k].V(0)->SetV();
				}
			} 
			if(a[0])
				boundary_idx.push_back((*m).face[k].V(0) - first);
			if(a[1])
				boundary_idx.push_back((*m).face[k].V(1) - first);
			if(a[2])
				boundary_idx.push_back((*m).face[k].V(2) - first);
		}
	}
	static void SelectPointsNearBoundary(CMeshO& mesh, int raidus)
	{
		Nanoknn nano_knn(mesh);
		nano_knn.buildNano();
		vector<int> boundary_idx;
		boundaryCheck(&mesh, boundary_idx);
		vcg::tri::UpdateFlags<CMeshO>::Clear(mesh);
		for(int i = 0 ; i < boundary_idx.size(); i++)
		{
			int idx = boundary_idx[i];
			vector<int> knnidx;
			nano_knn.findKNNNoSelf(mesh.vert.at(idx), raidus, knnidx);
			mesh.vert[idx].SetS();
			for(int i = 0 ; i < knnidx.size() ; i++)
			{
				mesh.vert[knnidx[i]].SetS();
				mesh.vert[knnidx[i]].C() = Color4b::Red;
			}
		}
	}
	static void boundaryFill(CMeshO& meshin, CMeshO& meshfilled)
	{
		CMeshO meshtemp;
		vcg::tri::Append<CMeshO, CMeshO>::MeshCopy(meshtemp, meshin);
		vector<pair<int, int>> boundarylinks;
		boundaryCheckEdge(meshtemp, boundarylinks);
		vector<vector<int>> loopsets;
		unordered_map<int, vector<int>> map_points;
		set<int> unvisited;
		for(auto a : boundarylinks)
		{
			map_points[a.first].push_back(a.second);
			map_points[a.second].push_back(a.first);
			unvisited.insert(a.first);
			unvisited.insert(a.second);
		}

		while(!unvisited.empty())
		{
			vector<int> memo;
			memo.push_back(*unvisited.begin());
			int idx = 0;
			bool isend = false;
			while(idx < memo.size())
			{
				int curr = memo[idx];
				unvisited.erase(curr);				
				int next1 = map_points[curr][0];
				int next2 = map_points[curr][1];

				if(unvisited.find(next1) != unvisited.end())
					memo.push_back(next1);
				if(unvisited.find(next2) != unvisited.end())
					memo.push_back(next2);
				idx++;
			}
			loopsets.push_back(memo);
			memo.clear();
		}

		vector<Point3d> Center(loopsets.size(), Point3d(0,0,0));
		for(int k = 0; k < loopsets.size(); k++)
		{
			for(int i = 0 ; i < loopsets[k].size(); i++)
			{
				Center[k] += meshtemp.vert.at(loopsets[k][i]).P();
			}
			Center[k] /= (double) loopsets[k].size();
		}

		unordered_map<int, int> mp_vidx2loops;
		for(int k = 0; k < loopsets.size(); k++)
		{
			for(int i = 0 ; i < loopsets[k].size(); i++)
			{
				mp_vidx2loops[loopsets[k][i]] = k;
			}
		}
		CMeshO::VertexIterator  vi = vcg::tri::Allocator<CMeshO>::AddVertices(meshtemp, Center.size());
		CMeshO::FaceIterator fi = vcg::tri::Allocator<CMeshO>::AddFaces(meshtemp, boundarylinks.size());
		assert(Center.size() < 100);
		CMeshO::VertexPointer ivp[100];
		for(int i = 0; i < Center.size(); i++)
		{
			ivp[i] = &*vi; vi->P() = CMeshO::CoordType(Center[i].X(),Center[i].Y(),Center[i].Z()); ++vi;
		}

		for(int i = 0; i < boundarylinks.size(); i++)
		{
			int idx1 = boundarylinks[i].first;
			int idx2 = boundarylinks[i].second;
			int idxc = mp_vidx2loops[idx1];
			fi->V(0) = ivp[idxc]; 
			fi->V(1) = &meshtemp.vert[idx2]; 
			fi->V(2) = &meshtemp.vert[idx1]; 
			++fi;
		}
		vcg::tri::Append<CMeshO, CMeshO>::MeshCopy(meshfilled, meshtemp);
		//Log::LogMesh_CMeshO(&meshfilled, "testboundary.ply");
	}

	static void MeshBridge(CMeshO& mesh, Color4b c)
	{
		vector<int> boundaryidx;
		CMeshO meshTemp;
		vcg::tri::Append<CMeshO, CMeshO>::MeshCopy(meshTemp, mesh);
		vector<pair<int, int>> memo;
		geohelp::boundaryCheckEdge(meshTemp, memo);
		vector<pair<int, int>> memo_select;
		for(auto i : memo)
		{
			if(meshTemp.vert[i.first].C().Equal(c) && meshTemp.vert[i.second].C().Equal(c))
			{
				memo_select.push_back(i);
			}
		}
		Point3d center(0, 0, 0);
		for(auto i : memo_select)
		{
			int idx1 = i.first;
			int idx2 = i.second;
			center += meshTemp.vert[idx1].P();
			center += meshTemp.vert[idx2].P();
		}
		center /= (memo_select.size() * 2);
		CMeshO::VertexIterator  vi = vcg::tri::Allocator<CMeshO>::AddVertices(meshTemp, 1);
		CMeshO::FaceIterator fi = vcg::tri::Allocator<CMeshO>::AddFaces(meshTemp, memo_select.size());
		CMeshO::VertexPointer ivp[1];
		ivp[0] = &*vi; vi->P() = CMeshO::CoordType(center.X(),center.Y(),center.Z());

		for(int i = 0; i < memo_select.size(); i++)
		{
			int idx1 = memo_select[i].first;
			int idx2 = memo_select[i].second;
			fi->V(0) = ivp[0]; 
			fi->V(1) = &meshTemp.vert[idx2]; 
			fi->V(2) = &meshTemp.vert[idx1]; 
			++fi;
		}
		CMeshO meshTemp2;
		vcg::tri::Append<CMeshO, CMeshO>::MeshCopy(meshTemp2, meshTemp);
		memo.clear();
		geohelp::boundaryCheckEdge(meshTemp2, memo);
		set<int> criticalboundary;
		for(auto i : memo)
		{
			int idx1 = i.first;
			int idx2 = i.second;
			if(meshTemp2.vert[idx1].C().Equal(c))
				criticalboundary.insert(i.first);
			if(meshTemp2.vert[idx2].C().Equal(c))
				criticalboundary.insert(i.second);
		}
		assert(criticalboundary.size() == 4);
		unordered_map<double, int> mp;
		//group critical boundary
		// fl fr bl br
		vector<Point3d> p;
		for(auto i : criticalboundary)
		{
			p.push_back(meshTemp2.vert[i].P());
			mp[p.back().X()] = i;
		}
		sort(p.begin(), p.end(), [](const Point3d& a, const Point3d& b)->bool{
			return a.Z() < b.Z();
		});
		if(p[0].X() < p[1].X())
			swap(p[0], p[1]);
		if(p[2].X() < p[3].X())
			swap(p[2], p[3]);
		vector<int> order(p.size());// fl fr bl br
		for(int i = 0 ; i < p.size(); i++)
		{
			order[i] = mp[p[i].X()];
		}
		CMeshO::FaceIterator fi2 = vcg::tri::Allocator<CMeshO>::AddFaces(meshTemp2, 2);
		int idx_center = 0;
		for(int i = meshTemp2.vert.size() - 1 ; i >= 0; i--)
		{
			if(meshTemp2.vert[i].P() == center)
			{
				idx_center = i;
				break;
			}
		}
#if 1
		fi2->V(0) = meshTemp2.vert.data() + idx_center; 
		fi2->V(1) = meshTemp2.vert.data() + order[0]; 
		fi2->V(2) = meshTemp2.vert.data() + order[2]; 
		fi2++;
		fi2->V(0) = meshTemp2.vert.data() + idx_center; 
		fi2->V(1) = meshTemp2.vert.data() + order[3]; 
		fi2->V(2) = meshTemp2.vert.data() + order[1]; 
#else
		fi2->V(0) = meshTemp2.vert.data() + idx_center; 
		fi2->V(1) = meshTemp2.vert.data() + order[2]; 
		fi2->V(2) = meshTemp2.vert.data() + order[0]; 
		fi2++;
		fi2->V(0) = meshTemp2.vert.data() + idx_center; 
		fi2->V(1) = meshTemp2.vert.data() + order[1]; 
		fi2->V(2) = meshTemp2.vert.data() + order[3]; 
#endif
		vcg::tri::Append<CMeshO, CMeshO>::MeshCopy(mesh, meshTemp2);
	}
};