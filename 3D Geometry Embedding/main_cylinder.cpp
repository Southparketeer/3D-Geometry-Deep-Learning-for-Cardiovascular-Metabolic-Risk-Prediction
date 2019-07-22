#include <windows.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <fstream>
#include <stdio.h>  
#include "Base.h"
#include "Helper.h"
#include "png_io.h"
#include "CylinderMap.h"

using namespace std;
using namespace Eigen;
using namespace vcg;

string ROOT_out = "D:TensorFlowTrain\\CMesh\\";
string ROOT_out_image = "D:TensorFlowTrain\\CCylinder\\";


void helper(CMeshO& mesh, int w, int h, bool* mask, const string& outputpath) {
	CMeshO mesh_temp;
	vcg::tri::Append<CMeshO, CMeshO>::MeshCopy(mesh_temp, mesh);
	for(int i = 0; i < mesh_temp.vert.size(); i++) {
		if(mask[i])  mesh_temp.vert[i].P().X() = sqrtf(-1);
	}
	vcg::tri::Clean<CMeshO>::RemoveDegenerateVertex(mesh_temp);
	CMeshO mesh_temp2;
	vcg::tri::Append<CMeshO, CMeshO>::MeshCopy(mesh_temp2, mesh_temp);
	vcg::Matrix44<double> rotation;
	rotation.SetZero();
	rotation[0][0] = 1; rotation[1][2] = 1; rotation[2][1] = -1; rotation[3][3] = 1;
	vcg::tri::UpdatePosition<CMeshO>::Matrix(mesh_temp2, rotation);
	CylinderMap cycmap;
	cycmap.init_Mesh(mesh_temp2, w, h, "temp\\Slice\\");
	cycmap.map_Distance();
	PNG_IO::depth_array_to_PNG(cycmap.cyc_img_distance.data(), outputpath.c_str(), cycmap.cyc_w, cycmap.cyc_h, 1e5);
}

int main() {
	int w = 256, h =256;
	CMeshO mesh;
	Log::LoadMesh("AverageShapeDepthMask_Trunk.ply", mesh);
	double* depth = new double[w * h];
	bool* mask = new bool[mesh.vert.size()];
	for(int i = 0; i < mesh.vert.size(); i++) {
		if(mesh.vert[i].C().Equal((Color4b)(Color4b::Green)))
			mask[i] = true;
		else
			mask[i] = false;
	}
	for(int k = 1; k <= 4300; k++) {
		CMeshO mesh_process;
		Log::LoadMesh(string(ROOT_out + "Mesh_" + to_string(k) + ".ply").c_str(), mesh_process);
		string output = string(ROOT_out_image + "Cylinder_Depth_" + to_string(k) + ".png");
		helper(mesh_process, w, h, mask, output);		
	}
	return 0;
}