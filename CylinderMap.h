#pragma once
#include "base.h"
#include "helper.h"
#include "GirthCompute.h"
#include <vcg/complex/algorithms/inertia.h>
#include <vcg/complex/algorithms/stat.h>
#include <vcg/complex/algorithms/smooth.h>
#include <vcg/complex/algorithms/update/curvature_fitting.h>
#include<vcg/complex/algorithms/update/curvature.h>
#include<vcg/complex/algorithms/update/normal.h>
#include <vcg/complex/algorithms/inertia.h>
#include <vcg/complex/algorithms/stat.h>
#include "Nanoknn.h"
using namespace Eigen;
using namespace vcg;

#define PI 3.14159265

class seg
{
public:
	void static segment(CMeshO& mesh, CMeshO& mesh_seg, double y_up, double y_down)
	{
		CMeshO mesh_temp;
		vcg::tri::Append<CMeshO, CMeshO>::MeshCopy(mesh_temp, mesh);
		for(int i = 0; i < mesh_temp.vert.size(); i++)
		{
			if(mesh_temp.vert[i].P().Y() > y_up || mesh_temp.vert[i].P().Y() < y_down)
			{
				mesh_temp.vert[i].P().X() = sqrtf(-1);
			}
		}
		vcg::tri::Clean<CMeshO>::RemoveDegenerateVertex(mesh_temp);
		vcg::tri::Append<CMeshO, CMeshO>::MeshCopy(mesh_seg, mesh_temp);
	}
};

class cyl
{
public:
	inline static Point3d cycoord_to_3d(double d, int cyc_x, int cyc_y, int cyc_w, int cyc_h)
	{
		Point3d p(0,0,0);
		p.Y() = (double) cyc_y / (double) cyc_h;
		double rad = ((double) cyc_x / (double) cyc_w) * PI * 2;
		p.X() = d * cos(rad);
		p.Z() = d * sin(rad);
		return p;
	}
	inline static int round(int x, int mod)
	{
		return (x + mod) % mod;
	}

	inline static int clamp(int x, int high, int low = 0)
	{
		return (x <= high && x >= 0) ? x : (x > high ? high : low);
	}
};

class CylinderMap
{
public:

	void init_Mesh(CMeshO& mesh, int w, int h, string subject_id = "");
	void map_Distance();
	void map_Normal();
	void map_Curvature();
	void map_Curvature(CMeshO& mesh); //for test;
	void fill_hole();
	void meshlize(CMeshO& mesh, int start_x = 0, int start_y = 0, int loc_w = 0, int loc_h = 0, int downsamplelevel = 0);

	int cyc_w;
	int cyc_h;
	string slicefname;
	vector<double> cyc_img_distance;
	vector<Point3d> cyc_img_normal;
	vector<Color4b> cyc_img_curvature;
	CMeshO mesh;
	CMeshO mesh_usample_H;
	CMeshO mesh_usample_L;
};

void CylinderMap::map_Distance()
{
	//Cylinder Image
	int num_layer = 8;
	double layer_thinkness = 1.0 / num_layer;
	cyc_img_distance.resize(cyc_w * cyc_h, 0);
	int kkk = 0;
	double margin = 1;
	for(int layer_idx = 0; layer_idx < num_layer; layer_idx++)
	{
		CMeshO Local_Mesh;
		seg::segment(mesh, Local_Mesh, (layer_idx + (1 + margin)) * layer_thinkness, (layer_idx - margin) * layer_thinkness);
		//Log::LogMesh_CMeshO(&Local_Mesh, string(slicefname + "layer" + to_string(layer_idx)+ ".ply").c_str());
		int lh = layer_idx * (double)(cyc_h / num_layer);
		vcg::tri::UpdateBounding<CMeshO>::Box(Local_Mesh);
		GirthCompute gc(Local_Mesh, Point3d(0, (layer_idx - margin) * layer_thinkness,0), Point3d(0,(layer_idx + (1 + margin)) * layer_thinkness, 0)); 
		for(int y = 0; y < cyc_h / num_layer; y++)
		{
			vector<pair<Point3d, Point3d>> pixel_intersect;
			gc.Compute(Point3d(0, layer_idx * (1.0 / num_layer) + y * (1.0 / cyc_h),0), 0, string(slicefname +"_s"+ to_string(kkk) + ".ply"), 1.0, &pixel_intersect);
			int loc_y = layer_idx * (cyc_h / num_layer) + y;	//local hight = lh + y;
			for(int x = 0; x < cyc_w; x++)
			{

				double rad = (x) * (360. / (double)cyc_w) * (PI/180); 
				Point2d ray_dir(cos(rad), sin(rad));
				double dis = INT_MIN;
				vector<double> memo;

				for(auto& p : pixel_intersect)
				{
					Point2d a(p.first.X(), p.first.Z());
					Point2d b(p.second.X(), p.second.Z());

					double s = a.Y() * (b.X() - a.X()) - a.X() * (b.Y() - a.Y());
					s /= ray_dir.Y() * (b.X() - a.X()) - ray_dir.X() * (b.Y() - a.Y());
					double t = 0.5;
					if(b.Y() != a.Y())
					{
						t = s * ray_dir.Y() - a.Y();
						t /= b.Y() - a.Y();
					}
					else
					{
						t = s * ray_dir.X() - a.X();
						t /= b.X() - a.X();
					}
					//s(ray_dir.x, ray_dir.y) = insect on line;
					if(t >= 0 && t <= 1) 
					{
						double d = (a + t * (b - a)).Norm();
						d = s < 0 ? -d : d;
						dis = max(d, dis);
						memo.push_back(d);
					}
				}
				cyc_img_distance[loc_y * cyc_w + x] = dis < 0 ? INT_MIN : dis;
			}
			kkk++;
		}
	}
	for(int x = 0; x < cyc_w; x++)
	{
		cyc_img_distance[0 * cyc_w + x] = INT_MIN;
		cyc_img_distance[(cyc_h - 1) * cyc_w + x] = INT_MIN;
	}
}

void CylinderMap::map_Curvature(CMeshO& mesh)
{
	CMeshO mesh_temp;
	vcg::tri::Append<CMeshO, CMeshO>::MeshCopy(mesh_temp, mesh);
	vcg::tri::Clean<CMeshO>::RemoveUnreferencedVertex(mesh_temp);
	tri::UpdateNormal<CMeshO>::NormalizePerVertex(mesh_temp);
	tri::UpdateCurvatureFitting<CMeshO>::computeCurvature(mesh_temp);
	tri::UpdateQuality<CMeshO>::VertexFromMeanCurvatureDir(mesh_temp);
	Histogramf H;
	tri::Stat<CMeshO>::ComputePerVertexQualityHistogram(mesh_temp,H);
	tri::UpdateColor<CMeshO>::PerVertexQualityRamp(mesh_temp,H.Percentile(0.1f),H.Percentile(0.9f));
	CMeshO::VertexIterator vi = mesh_temp.vert.begin();
	for(auto &i : mesh.vert)
	{
		if(i.P() != Point3d(0,0,0))
		{
			i.Q() = vi->Q();
			i.C() = vi->C();
			vi++;
		}
		else
		{
			i.Q() = 0;
		}
	}
	//Log::LogMesh_CMeshO_Q(&mesh, "test.ply");
	double scale = sqrtf(cyc_w * cyc_h / mesh.vert.size());
	for(int y = 0; y < cyc_h; y++)
	{
		for(int x = 0; x < cyc_w; x++)
		{
			int hit = floor(y / scale + 0.5) * (cyc_w / scale) + floor(x / scale + 0.5);
			if(hit > mesh.vert.size())
				continue;
			cyc_img_curvature[y * cyc_w + x] = mesh.vert[hit].C();
		}
	}
}
void CylinderMap::map_Curvature()
{
	if(mesh_usample_L.vert.size() == 0 && mesh_usample_H.vert.size() == 0)
	{
		this->meshlize(this->mesh_usample_H, 0, 0, cyc_w, cyc_h, 0);
		this->meshlize(this->mesh_usample_L, 0, 0, cyc_w, cyc_h, 2);
	}
	map_Curvature(this->mesh_usample_L);
}

void CylinderMap::map_Normal()
{
	cyc_img_normal.resize(cyc_w * cyc_h, Point3d(0,0,0));
	for(int y = 0; y < cyc_h; y++)
	{
		for(int x = 0; x < cyc_w; x++)
		{
			int base = y * cyc_w + x;
			double d = cyc_img_distance[base];
			if(d <= 0)
				continue;
			Point2d up = y == cyc_h - 1 ? Point2d(x, y) : Point2d(x, y + 1);
			Point2d dw = y == 0 ? Point2d(x, y) : Point2d(x, y - 1);
			Point2d lf(cyl::round(x + 1 , cyc_w), y);
			Point2d rt(cyl::round(x - 1,  cyc_w), y);
			double d_up = cyc_img_distance[up.Y() * cyc_w + up.X()];
			double d_dw = cyc_img_distance[dw.Y() * cyc_w + dw.X()];
			double d_lf = cyc_img_distance[lf.Y() * cyc_w + lf.X()];
			double d_rt = cyc_img_distance[rt.Y() * cyc_w + rt.X()];

			if(d_up == 0 || d_dw == 0 || d_lf == 0 || d_rt == 0 || d == 0)
				continue;
			Point3d p_up = cyl::cycoord_to_3d(d_up, up.X(), up.Y(), cyc_w, cyc_h);
			Point3d p_dw = cyl::cycoord_to_3d(d_dw, dw.X(), dw.Y(), cyc_w, cyc_h);
			Point3d p_lf = cyl::cycoord_to_3d(d_lf, lf.X(), lf.Y(), cyc_w, cyc_h);
			Point3d p_rt = cyl::cycoord_to_3d(d_rt, rt.X(), rt.Y(), cyc_w, cyc_h);
			Point3d p_dx = p_up - p_dw;
			Point3d p_dy = p_lf - p_rt;
			Vector3f eg_x(p_dx.X(), p_dx.Y(), p_dx.Z());
			Vector3f eg_y(p_dy.X(), p_dy.Y(), p_dy.Z());
			Vector3f eg_n = eg_x.cross(eg_y);
			eg_n.normalize();
			cyc_img_normal[base] = Point3d(eg_n.x(), eg_n.y(), eg_n.z());
		}
	}
}

void CylinderMap::init_Mesh(CMeshO& meshIN, int w, int h, string slice_debug)
{
	slicefname = slice_debug;
	cyc_w = w;
	cyc_h = h;
	cyc_img_distance.resize(w * h, 0);
	cyc_img_normal.resize(w * h, Point3d(0,0,0));
	cyc_img_curvature.resize(w * h);
	vcg::tri::Append<CMeshO, CMeshO>::MeshCopy(mesh, meshIN);
	vcg::tri::UpdateBounding<CMeshO>::Box(mesh);
	//-------------------------------------------------------------------------
		double height = mesh.bbox.max.Y() - mesh.bbox.min.Y();
		Point3d Top = Point3d(mesh.bbox.Center().X(), mesh.bbox.max.Y(), mesh.bbox.Center().Z());
		Point3d Down = Point3d(mesh.bbox.Center().X(), mesh.bbox.min.Y(), mesh.bbox.Center().Z());

		GirthCompute gc(mesh, Down, Top); //Root -> Child
		vector<pair<Point3d, Point3d>> Top_intersection, Down_intersecion;
		gc.Compute(Top  - Point3d(0, 0.2 * height , 0), 0, string(slicefname + "_Top.ply").c_str(), 1.0, &Top_intersection);
		gc.Compute(Down + Point3d(0, 0.2  * height, 0), 0, string(slicefname + "_Down.ply").c_str(), 1.0, &Down_intersecion);
		Point3d NewCenter(0,0,0);
		for(auto i: Top_intersection)
			NewCenter += (i.first + i.second);
		for(auto i: Down_intersecion)
			NewCenter += (i.first + i.second);
		NewCenter /= (Top_intersection.size() + Down_intersecion.size())* 2;
		NewCenter.Y() = mesh.bbox.min.Y();
		vcg::tri::UpdatePosition<CMeshO>::Translate(mesh, NewCenter * -1);
		vcg::tri::UpdatePosition<CMeshO>::Scale(mesh, 1. / (mesh.bbox.max.Y() - mesh.bbox.min.Y()));
		//Log::LogMesh_CMeshO(&mesh, "test.ply");
}

void CylinderMap::meshlize(CMeshO& m_mesh, int sx, int sy, int lw, int lh, int downsamplelevel)
{
	//anti clockwise
	//a----b
	//|	   |
	//d----c
	if(sx != 0 && sy != 0)
		downsamplelevel = 0;
	lw >>= downsamplelevel;
	lh >>= downsamplelevel;
	int w = (cyc_w >> downsamplelevel);
	int h = (cyc_h >> downsamplelevel);
	vector<double>* img_distance_ptr = NULL;
	vector<double> temp_img_distance;
	if(downsamplelevel == 0)
		img_distance_ptr = &cyc_img_distance;
	else
	{
		temp_img_distance.resize(w * h, FLT_MAX);
		for(int j = 0; j < h; j++)
		{
			for(int i = 0 ; i < w; i++)
			{
				temp_img_distance[j * w + i] = this->cyc_img_distance[(j << downsamplelevel) * cyc_w + (i << downsamplelevel)];
			}
		}
		img_distance_ptr = &temp_img_distance;
	}

	vector<Point3d> buff_vertex(lw * lh, Point3d(0,0,0));
	vector<Point3i> buff_idx;
	for(int y = sy, j = 0 ; y < sy + lh; y++, j++)
	{
		for(int x = sx, i = 0; x < sx + lw; x++, i++)
		{
			int b_global = y * w + cyl::round(x, w);
			int b_local  = j * lw + i;
			bool mark_a = (*img_distance_ptr)[b_global] > 0 && (*img_distance_ptr)[b_global] < 0.5;
			if(mark_a)
			{
				buff_vertex[b_local] = cyl::cycoord_to_3d((*img_distance_ptr)[b_global], cyl::round(x, w), y, w, h);
			}
			else
			{
				buff_vertex[b_local] = Point3d(0, 0, 0);
			}
		}
	}

	for(int y = sy, j = 0; y < sy + lh - 1; y++, j++)
	{
		for(int x = sx, i = 0; x < sx + lw; x++, i++)
		{
			int b_global = y * w + cyl::round(x, w);
			int n = 0;
			bool mark_a = buff_vertex[j * lw + i] != Point3d(0,0,0); if(mark_a) n++;
			bool mark_b = buff_vertex[j * lw + cyl::round(i + 1, w)] != Point3d(0,0,0); if(mark_b) n++;
			bool mark_c = buff_vertex[(j + 1) * lw + cyl::round(i + 1, w)] != Point3d(0,0,0); if(mark_c) n++;
			bool mark_d = buff_vertex[(j + 1) * lw + i] != Point3d(0,0,0); if(mark_d) n++;
			if(n <= 2) 
				continue;
			int idx_a = j * lw + i;
			int idx_b = j * lw + cyl::round(i + 1, w);
			int idx_c = (j + 1) * lw + cyl::round(i + 1, w);
			int idx_d = (j + 1) * lw + i;
			if(n == 4)
			{
				// a - b - c
				buff_idx.push_back(Point3i(idx_a, idx_c, idx_b));
				// a - c - d
				buff_idx.push_back(Point3i(idx_a, idx_d, idx_c));
			}
			if(n == 3)
			{
				if(!mark_a)
				{
					//b - c - d
					buff_idx.push_back(Point3i(idx_b, idx_d, idx_c));
				}
				if(!mark_b)
				{
					//a - c - d
					buff_idx.push_back(Point3i(idx_a, idx_d, idx_c));
				}
				if(!mark_c)
				{
					// a - b - d
					buff_idx.push_back(Point3i(idx_a, idx_d, idx_b));
				}
				if(!mark_d)
				{
					// a - b - c
					buff_idx.push_back(Point3i(idx_a, idx_c, idx_b));
				}
			}
		}
	}

	CMeshO m;
	CMeshO::VertexIterator vi = vcg::tri::Allocator<CMeshO>::AddVertices(m, buff_vertex.size());
	CMeshO::FaceIterator fi = vcg::tri::Allocator<CMeshO>::AddFaces(m, buff_idx.size());
	vector<CMeshO::VertexPointer> ivp(buff_vertex.size(), NULL);
	for(int i = 0 ; i < buff_vertex.size(); i++)
	{
		ivp[i] = &*vi;
		vi->P() = CMeshO::CoordType(buff_vertex[i].X(), buff_vertex[i].Y(), buff_vertex[i].Z());
		++vi;
	}
	for(int i = 0 ;i < buff_idx.size(); i++)
	{
		fi->V(0) = ivp[buff_idx[i].X()];
		fi->V(1) = ivp[buff_idx[i].Y()];
		fi->V(2) = ivp[buff_idx[i].Z()];
		fi++;
	}
	vcg::tri::Append<CMeshO, CMeshO>::MeshCopy(m_mesh, m);
}
