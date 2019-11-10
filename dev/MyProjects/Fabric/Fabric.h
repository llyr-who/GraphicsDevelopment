//***************************************************************************************
// Fabric.h by llyr-who (C) 2011 All Rights Reserved.
//
// Performs the calculations for the wave simulation.  After the simulation has been
// updated, the client must copy the current solution into vertex buffers for rendering.
// This class only does the calculations, it does not do any drawing.
//***************************************************************************************

#ifndef FABRIC_H
#define FABRIC_H

#include <vector>
#include <DirectXMath.h>
class Fabric
{
public:
	Fabric(std::size_t m, std::size_t n, float ddx, float ddt, float spring1, float spring2, float damp1, float damp2, float M);
	std::size_t RowCount() const;
	std::size_t ColumnCount() const;
	std::size_t TriangleCount() const;
	std::size_t VertexCount() const;
	float Width()const;
	float Depth() const;

	// this returns the solution at the ith grid point
	const DirectX::XMFLOAT3& operator[](int i)const { return currPos[i]; }

	// Returns the solution normal at the ith grid point.
	const DirectX::XMFLOAT3& Normal(int i)const { return normals[i]; }
	// Returns the unit tangent vector at the ith grid point
	const DirectX::XMFLOAT3& Tangent(int i)const { return tangents[i]; }
	// Returns the unit bitangent vector at the ith grid point
	const DirectX::XMFLOAT3& Bitangent(int i)const { return bitangents[i]; }

	void Update(float dt, float windX, float windY, float windZ);
private:
	std::size_t numRows;
	std::size_t numCols;

	std::size_t vertexCount;
	std::size_t triangleCount;

	float dt; //time step
	float dx; //spatial step
	float mass;
	//all we want is the magnitude in the y-direction.
	//as gravity always points in the neg y direction.
	float gravity;

	//some ability to modify the spring constant parameters
	//should be added at some point.
	float shortDamp;
	float shortSpring;
	float longDamp;
	float longSpring;
	std::vector<DirectX::XMFLOAT3> prevPos;
	std::vector<DirectX::XMFLOAT3> currPos;
	std::vector<DirectX::XMFLOAT3> velocity;
	std::vector<DirectX::XMFLOAT3> normals;
	std::vector<DirectX::XMFLOAT3> tangents;
	std::vector<DirectX::XMFLOAT3> bitangents;
	std::vector<DirectX::XMFLOAT3> force;
};

#endif