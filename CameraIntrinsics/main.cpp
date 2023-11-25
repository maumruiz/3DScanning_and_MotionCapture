#include <iostream>
#include <fstream>
#include <array>

#include "Eigen.h"
#include "VirtualSensor.h"

struct Vertex
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	// position stored as 4 floats (4th component is supposed to be 1.0)
	Vector4f position;
	// color stored as 4 unsigned char
	Vector4uc color;
};

bool isValidFace(Vertex* vertices, unsigned v1, unsigned v2, unsigned v3, float edgeThreshold) {
	return ( (vertices[v1].position - vertices[v2].position).norm() < edgeThreshold &&
			  (vertices[v1].position - vertices[v3].position).norm() < edgeThreshold &&
			  (vertices[v2].position - vertices[v3].position).norm() < edgeThreshold
		);
}

bool WriteMesh(Vertex* vertices, unsigned int width, unsigned int height, const std::string& filename)
{
	float edgeThreshold = 0.01f; // 1cm

	// TODO 2: use the OFF file format to save the vertices grid (http://www.geomview.org/docs/html/OFF.html)
	// - have a look at the "off_sample.off" file to see how to store the vertices and triangles
	// - for debugging we recommend to first only write out the vertices (set the number of faces to zero)
	// - for simplicity write every vertex to file, even if it is not valid (position.x() == MINF) (note that all vertices in the off file have to be valid, thus, if a point is not valid write out a dummy point like (0,0,0))
	// - use a simple triangulation exploiting the grid structure (neighboring vertices build a triangle, two triangles per grid cell)
	// - you can use an arbitrary triangulation of the cells, but make sure that the triangles are consistently oriented
	// - only write triangles with valid vertices and an edge length smaller then edgeThreshold

	// TODO: Get number of vertices
	unsigned int nVertices = width * height;
	
	// TODO: Determine number of valid faces
	unsigned nFaces = 0;
	std::vector<std::array<unsigned int, 3>> faces;

	for(unsigned int row = 0; row < height-1; row++){
		for(unsigned int col = 0; col < width-1; col++) {
			unsigned v1 = row*width+col;
			unsigned v2 = (row+1)*width + col;
			unsigned v3 = row*width + col + 1;
			unsigned v4 = (row+1)*width + col + 1;

			if(isValidFace(vertices, v1, v2, v3, edgeThreshold)) {
				faces.push_back(std::array<unsigned int, 3>({v1, v2, v3}));
				nFaces++;
			}

			if(isValidFace(vertices, v2, v3, v4, edgeThreshold)) {
				faces.push_back(std::array<unsigned int, 3>({v2, v3, v4}));
				nFaces++;
			}
		}
	}

	// Write off file
	std::ofstream outFile(filename);
	if (!outFile.is_open()) return false;

	// write header
	outFile << "COFF" << std::endl;
	outFile << nVertices << " " << nFaces << " 0" << std::endl;

	// TODO: save vertices
	for(unsigned int i = 0; i < width * height; i++) {
		if(vertices[i].position[0] == MINF)
			outFile << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << std::endl;
		else {
			outFile << vertices[i].position[0] << " " << vertices[i].position[1] << " " << vertices[i].position[2] << " "
				<< (unsigned int) vertices[i].color[0] << " " << (unsigned int) vertices[i].color[1] << " "
				<< (unsigned int) vertices[i].color[2] << " " << (unsigned int) vertices[i].color[3] << std::endl;
		}
	}
	

	// TODO: save valid faces
	for(unsigned int i = 0; i < nFaces; i++) {
		outFile << 3 << " " << faces[i][0] << " " << faces[i][1] << " " << faces[i][2] << std::endl;
	}

	// close file
	outFile.close();

	return true;
}

int main()
{
	std::cout << "Starting program!" << std::endl;

	// Make sure this path points to the data folder
	std::string filenameIn = "../../data/rgbd_dataset_freiburg1_xyz/";
	std::string filenameBaseOut = "mesh_";

	// load video
	std::cout << "Initialize virtual sensor..." << std::endl;
	VirtualSensor sensor;
	if (!sensor.Init(filenameIn))
	{
		std::cout << "Failed to initialize the sensor!\nCheck file path!" << std::endl;
		return -1;
	}

	// convert video to meshes
	while (sensor.ProcessNextFrame())
	{
		// get ptr to the current depth frame
		// depth is stored in row major (get dimensions via sensor.GetDepthImageWidth() / GetDepthImageHeight())
		float* depthMap = sensor.GetDepth();
		// get ptr to the current color frame
		// color is stored as RGBX in row major (4 byte values per pixel, get dimensions via sensor.GetColorImageWidth() / GetColorImageHeight())
		BYTE* colorMap = sensor.GetColorRGBX();

		// get depth intrinsics
		Matrix3f depthIntrinsics = sensor.GetDepthIntrinsics();
		float fX = depthIntrinsics(0, 0);
		float fY = depthIntrinsics(1, 1);
		float cX = depthIntrinsics(0, 2);
		float cY = depthIntrinsics(1, 2);

		// compute inverse depth extrinsics
		Matrix4f depthExtrinsicsInv = sensor.GetDepthExtrinsics().inverse();

		Matrix4f trajectory = sensor.GetTrajectory();
		Matrix4f trajectoryInv = sensor.GetTrajectory().inverse();

		// TODO 1: back-projection
		// write result to the vertices array below, keep pixel ordering!
		// if the depth value at idx is invalid (MINF) write the following values to the vertices array
		// vertices[idx].position = Vector4f(MINF, MINF, MINF, MINF);
		// vertices[idx].color = Vector4uc(0,0,0,0);
		// otherwise apply back-projection and transform the vertex to world space, use the corresponding color from the colormap
		Vertex* vertices = new Vertex[sensor.GetDepthImageWidth() * sensor.GetDepthImageHeight()];
		
		for(size_t u = 0; u < sensor.GetDepthImageHeight(); u++) {
			for(size_t v = 0; v < sensor.GetDepthImageWidth(); v++) {
				size_t idx = u * sensor.GetDepthImageWidth() + v;
				float Z = depthMap[idx];
				auto color = colorMap + idx*4;
				if(Z == MINF) {
					vertices[idx].position = Vector4f({MINF, MINF, MINF, MINF});
					vertices[idx].color = Vector4uc({0,0,0,0});
				}
				else {
					// Back-project the pixels of the depth map to the camera space of the depth camera (using the intrinsics).
					// p_image = Vector3f(v *)
					float X = (v - cX) / fX * Z;
					float Y = (u - cY) / fY * Z;
					Vector4f point_3d = Vector4f(X, Y, Z, 1.0);

					// Transform the back-projected 3D points to world space 
					point_3d = depthExtrinsicsInv * point_3d;
					point_3d = trajectoryInv * point_3d;
					vertices[idx].position = point_3d;
					vertices[idx].color = Vector4uc(color[0], color[1], color[2], color[3]);
				}
			}
		}

		// write mesh file
		std::stringstream ss;
		ss << filenameBaseOut << sensor.GetCurrentFrameCnt() << ".off";
		if (!WriteMesh(vertices, sensor.GetDepthImageWidth(), sensor.GetDepthImageHeight(), ss.str()))
		{
			std::cout << "Failed to write mesh!\nCheck file path!" << std::endl;
			return -1;
		}

		// free mem
		delete[] vertices;
	}

	return 0;
}
