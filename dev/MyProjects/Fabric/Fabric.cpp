
#include"Fabric.h"
#include"../../Common/MathHelper.h"

#include <ppl.h>
#include <algorithm>
#include <vector>
#include <cassert>

using DirectX::XMFLOAT3;

Fabric::Fabric(std::size_t m, std::size_t n, float ddx, float ddt, float spring1, float spring2, float damp1, float damp2, float M)
{
	mass = M;
	numRows = m;
	numCols = n;

	vertexCount = m * n;
	triangleCount = (m - 1) * (n - 1) * 2;

	dt = ddt;
	dx = ddx;

	shortDamp = damp1;
	longDamp = damp2;
	shortSpring = spring1;
	longSpring = spring2;

	prevPos.resize(m * n);
	currPos.resize(m * n);
	velocity.resize(m * n);
	force.resize(m * n);
	normals.resize(m * n);
	tangents.resize(m * n);
	bitangents.resize(m * n);

	// Generate grid vertices in system memory.

	float halfWidth = (n - 1) * dx * 0.5f;
	float halfDepth = (m - 1) * dx * 0.5f;
	for (std::size_t i = 0; i < m; ++i)
	{
		float z = halfDepth - i * dx;
		for (std::size_t j = 0; j < n; ++j)
		{
			float x = -halfWidth + j * dx;

			prevPos[i * n + j] = XMFLOAT3(x, 0.1 * sin(x * z), z);
			currPos[i * n + j] = XMFLOAT3(x, 0.1 * sin(x * z), z);
			velocity[i * n + j] = XMFLOAT3(0, 0, 0);
			normals[i * n + j] = XMFLOAT3(0, 1, 0);
		}
	}
}


std::size_t Fabric::RowCount()const
{
	return numRows;
}

std::size_t Fabric::ColumnCount()const
{
	return numCols;
}

std::size_t Fabric::VertexCount()const
{
	return vertexCount;
}

std::size_t Fabric::TriangleCount()const
{
	return triangleCount;
}

float Fabric::Width()const
{
	return numCols * dx;
}

float Fabric::Depth()const
{
	return numRows * dx;
}

void Fabric::Update(float ddt, float windX, float windY, float windZ)
{
	static float t = 0;
	t += ddt;
	std::size_t n = numCols;
	std::size_t m = numRows;
	if (t >= dt)
	{

		for (std::size_t j = 0; j < m; ++j)
		{
			for (std::size_t i = 0; i < n; ++i)
			{
				force[j * n + i].x = 0;
				force[j * n + i].y = 0;
				force[j * n + i].z = 0;
			}
		}


		// Wind update function
		// does this make sence for the wind update?
		concurrency::parallel_for(0, int(m), [this, &windX, &windY, &windZ, &n](int j)
			{
				for (std::size_t i = 0; i < n; i++)
				{
					std::size_t j_times_n_plus_i = j * n + i;
					float WFx, WFy, WFz;
					// if the wind and the velocity are in opposite directions
					// it would make sence for the particle to be unaffected.
					WFx = normals[j_times_n_plus_i].x * (windX + velocity[j_times_n_plus_i].x);
					WFy = normals[j_times_n_plus_i].y * (windY + velocity[j_times_n_plus_i].y);
					WFz = normals[j_times_n_plus_i].z * (windZ + velocity[j_times_n_plus_i].z);
					force[j_times_n_plus_i].x += wind_infl * WFx;
					force[j_times_n_plus_i].y += mass * gravity + wind_infl * WFx;
					force[j_times_n_plus_i].z += wind_infl * WFx;

				}
			});

		//we do the double links first
		concurrency::parallel_for(0, int(m - 2), [this, &n](int j)
			{
				for (std::size_t i = 0; i < n - 2; i++)
				{
					// not sure how much this "optimisation" makes much of
					// a difference - if at all
					std::size_t j_plus_2_times_n_plus_i = (j + 2) * n + i;
					std::size_t j_times_n_plus_i = j * n + i;
					std::size_t j_times_n_plus_i_plus_2 = j * n + i + 2;

					float F1x, F1y, F1z;
					float F2x, F2y, F2z;
					float diffx;
					float diffy;
					float diffz;
					float velDiffx;
					float velDiffy;
					float velDiffz;
					float diffNorm;
					float k;

					diffx = currPos[j_plus_2_times_n_plus_i].x - currPos[j_times_n_plus_i].x;
					diffy = currPos[j_plus_2_times_n_plus_i].y - currPos[j_times_n_plus_i].y;
					diffz = currPos[j_plus_2_times_n_plus_i].z - currPos[j_times_n_plus_i].z;
					diffNorm = sqrt(diffx * diffx + diffy * diffy + diffz * diffz);
					k = longSpring * (diffNorm - 2 * dx) * (1 / diffNorm);
					//now force due to damper
					velDiffx = velocity[j_plus_2_times_n_plus_i].x - velocity[j_times_n_plus_i].x;
					velDiffy = velocity[j_plus_2_times_n_plus_i].y - velocity[j_times_n_plus_i].y;
					velDiffz = velocity[j_plus_2_times_n_plus_i].z - velocity[j_times_n_plus_i].z;

					F1x = k * diffx + longDamp * velDiffx;
					F1y = k * diffy + longDamp * velDiffy;
					F1z = k * diffz + longDamp * velDiffz;

					diffx = currPos[j_times_n_plus_i_plus_2].x - currPos[j_times_n_plus_i].x;
					diffy = currPos[j_times_n_plus_i_plus_2].y - currPos[j_times_n_plus_i].y;
					diffz = currPos[j_times_n_plus_i_plus_2].z - currPos[j_times_n_plus_i].z;
					diffNorm = sqrt(diffx * diffx + diffy * diffy + diffz * diffz);
					k = longSpring * (diffNorm - 2 * dx) * (1 / diffNorm);
					//now force due to damper
					velDiffx = velocity[j_times_n_plus_i_plus_2].x - velocity[j_times_n_plus_i].x;
					velDiffy = velocity[j_times_n_plus_i_plus_2].y - velocity[j_times_n_plus_i].y;
					velDiffz = velocity[j_times_n_plus_i_plus_2].z - velocity[j_times_n_plus_i].z;

					F2x = k * diffx + longDamp * velDiffx;
					F2y = k * diffy + longDamp * velDiffy;
					F2z = k * diffz + longDamp * velDiffz;

					force[j_plus_2_times_n_plus_i].x -= F1x;
					force[j_plus_2_times_n_plus_i].y -= F1y;
					force[j_plus_2_times_n_plus_i].z -= F1z;

					force[j_times_n_plus_i].x += F1x;
					force[j_times_n_plus_i].y += F1y;
					force[j_times_n_plus_i].z += F1z;
					force[j_times_n_plus_i].x += F2x;
					force[j_times_n_plus_i].y += F2y;
					force[j_times_n_plus_i].z += F2z;

					force[j_times_n_plus_i_plus_2].x -= F2x;
					force[j_times_n_plus_i_plus_2].y -= F2y;
					force[j_times_n_plus_i_plus_2].z -= F2z;
				}
			});

		for (std::size_t i = 0; i < n - 2; i++)
		{


			float F2x, F2y, F2z;
			float diffx;
			float diffy;
			float diffz;
			float velDiffx;
			float velDiffy;
			float velDiffz;
			float diffNorm;
			float k;


			diffx = currPos[(m - 1) * n + i + 2].x - currPos[(m - 1) * n + i].x;
			diffy = currPos[(m - 1) * n + i + 2].y - currPos[(m - 1) * n + i].y;
			diffz = currPos[(m - 1) * n + i + 2].z - currPos[(m - 1) * n + i].z;
			// is this sqrt necc? - probably not
			diffNorm = sqrt(diffx * diffx + diffy * diffy + diffz * diffz);
			k = longSpring * (diffNorm - 2 * dx) * (1 / diffNorm);
			//now force due to damper
			velDiffx = velocity[(m - 1) * n + i + 2].x - velocity[(m - 1) * n + i].x;
			velDiffy = velocity[(m - 1) * n + i + 2].y - velocity[(m - 1) * n + i].y;
			velDiffz = velocity[(m - 1) * n + i + 2].z - velocity[(m - 1) * n + i].z;

			F2x = k * diffx + longDamp * velDiffx;
			F2y = k * diffy + longDamp * velDiffy;
			F2z = k * diffz + longDamp * velDiffz;


			force[(m - 1) * n + i + 2].x -= F2x;
			force[(m - 1) * n + i + 2].y -= F2y;
			force[(m - 1) * n + i + 2].z -= F2z;

			force[(m - 1) * n + i].x += F2x;
			force[(m - 1) * n + i].y += F2y;
			force[(m - 1) * n + i].z += F2z;



		}

		for (std::size_t j = 0; j < m - 2; j++)
		{
			float F1x, F1y, F1z;

			float diffx;
			float diffy;
			float diffz;
			float velDiffx;
			float velDiffy;
			float velDiffz;
			float diffNorm;
			float k;

			diffx = currPos[(j + 2) * n + n - 1].x - currPos[j * n + n - 1].x;
			diffy = currPos[(j + 2) * n + n - 1].y - currPos[j * n + n - 1].y;
			diffz = currPos[(j + 2) * n + n - 1].z - currPos[j * n + n - 1].z;
			diffNorm = sqrt(diffx * diffx + diffy * diffy + diffz * diffz);
			k = longSpring * (diffNorm - 2 * dx) * (1 / diffNorm);
			//now force due to damper
			velDiffx = velocity[(j + 2) * n + n - 1].x - velocity[j * n + n - 1].x;
			velDiffy = velocity[(j + 2) * n + n - 1].y - velocity[j * n + n - 1].y;
			velDiffz = velocity[(j + 2) * n + n - 1].z - velocity[j * n + n - 1].z;

			F1x = k * diffx + longDamp * velDiffx;
			F1y = k * diffy + longDamp * velDiffy;
			F1z = k * diffz + longDamp * velDiffz;

			force[(j + 2) * n + n - 1].x -= F1x;
			force[(j + 2) * n + n - 1].y -= F1y;
			force[(j + 2) * n + n - 1].z -= F1z;


			force[j * n + n - 1].x += F1x;
			force[j * n + n - 1].y += F1y;
			force[j * n + n - 1].z += F1z;

		}

		concurrency::parallel_for(0, int(m - 1), [this, &n](int j)
			{
				for (std::size_t i = 0; i < n - 1; i++)
				{
					float F1x, F1y, F1z;
					float F2x, F2y, F2z;
					float F3x, F3y, F3z;
					float F4x, F4y, F4z;
					float diffx;
					float diffy;
					float diffz;
					float velDiffx;
					float velDiffy;
					float velDiffz;
					float diffNorm;
					float k;
					//the "from left to right" diagonal connection in our computational molecule
					//force due to spring first
					// NOTE THE SQUARE ROOT IN DIAG CONNECTIONS

					diffx = currPos[(j + 1) * n + (i + 1)].x - currPos[j * n + i].x;
					diffy = currPos[(j + 1) * n + (i + 1)].y - currPos[j * n + i].y;
					diffz = currPos[(j + 1) * n + (i + 1)].z - currPos[j * n + i].z;
					diffNorm = sqrt(diffx * diffx + diffy * diffy + diffz * diffz);
					k = shortSpring * (diffNorm - MathHelper::sqrt_2 * dx) * (1 / diffNorm);
					//now force due to damper
					velDiffx = velocity[(j + 1) * n + (i + 1)].x - velocity[j * n + i].x;
					velDiffy = velocity[(j + 1) * n + (i + 1)].y - velocity[j * n + i].y;
					velDiffz = velocity[(j + 1) * n + (i + 1)].z - velocity[j * n + i].z;

					F1x = k * diffx + shortDamp * velDiffx;
					F1y = k * diffy + shortDamp * velDiffy;
					F1z = k * diffz + shortDamp * velDiffz;

					//the "from right to left" diagonal connection in our computational molecule
					//force due to spring first
					// NOTE THE SQUARE ROOT IN DIAG CONNECTIONS

					diffx = currPos[(j + 1) * n + i].x - currPos[j * n + (i + 1)].x;
					diffy = currPos[(j + 1) * n + i].y - currPos[j * n + (i + 1)].y;
					diffz = currPos[(j + 1) * n + i].z - currPos[j * n + (i + 1)].z;
					diffNorm = sqrt(diffx * diffx + diffy * diffy + diffz * diffz);
					k = shortSpring * (diffNorm - MathHelper::sqrt_2 * dx) * (1 / diffNorm);
					//now force due to damper
					velDiffx = velocity[(j + 1) * n + i].x - velocity[j * n + (i + 1)].x;
					velDiffy = velocity[(j + 1) * n + i].y - velocity[j * n + (i + 1)].y;
					velDiffz = velocity[(j + 1) * n + i].z - velocity[j * n + (i + 1)].z;

					F2x = k * diffx + shortDamp * velDiffx;
					F2y = k * diffy + shortDamp * velDiffy;
					F2z = k * diffz + shortDamp * velDiffz;

					//non-diag connection "up to down" horizontal component

					diffx = currPos[(j + 1) * n + i].x - currPos[j * n + i].x;
					diffy = currPos[(j + 1) * n + i].y - currPos[j * n + i].y;
					diffz = currPos[(j + 1) * n + i].z - currPos[j * n + i].z;
					diffNorm = sqrt(diffx * diffx + diffy * diffy + diffz * diffz);
					k = shortSpring * (diffNorm - dx) * (1 / diffNorm);
					//now force due to damper
					velDiffx = velocity[(j + 1) * n + i].x - velocity[j * n + i].x;
					velDiffy = velocity[(j + 1) * n + i].y - velocity[j * n + i].y;
					velDiffz = velocity[(j + 1) * n + i].z - velocity[j * n + i].z;

					F4x = k * diffx + shortDamp * velDiffx;
					F4y = k * diffy + shortDamp * velDiffy;
					F4z = k * diffz + shortDamp * velDiffz;

					// this is the "from dleft to right" horizontal component

					diffx = currPos[j * n + (i + 1)].x - currPos[j * n + i].x;
					diffy = currPos[j * n + (i + 1)].y - currPos[j * n + i].y;
					diffz = currPos[j * n + (i + 1)].z - currPos[j * n + i].z;
					diffNorm = sqrt(diffx * diffx + diffy * diffy + diffz * diffz);
					k = shortSpring * (diffNorm - dx) * (1 / diffNorm);
					//now force due to damper
					velDiffx = velocity[j * n + (i + 1)].x - velocity[j * n + i].x;
					velDiffy = velocity[j * n + (i + 1)].y - velocity[j * n + i].y;
					velDiffz = velocity[j * n + (i + 1)].z - velocity[j * n + i].z;

					F3x = k * diffx + shortDamp * velDiffx;
					F3y = k * diffy + shortDamp * velDiffy;
					F3z = k * diffz + shortDamp * velDiffz;
					/*
					*here is where we add the force to the force array
					*/


					force[j * n + i].x += F1x;
					force[j * n + i].y += F1y;
					force[j * n + i].z += F1z;

					force[j * n + i].x += F3x;
					force[j * n + i].y += F3y;
					force[j * n + i].z += F3z;

					force[j * n + i].x += F4x;
					force[j * n + i].y += F4y;
					force[j * n + i].z += F4z;

					force[(j + 1) * n + (i + 1)].x -= F1x;
					force[(j + 1) * n + (i + 1)].y -= F1y;
					force[(j + 1) * n + (i + 1)].z -= F1z;

					force[j * n + (i + 1)].x -= F3x;
					force[j * n + (i + 1)].y -= F3y;
					force[j * n + (i + 1)].z -= F3z;

					force[j * n + (i + 1)].x += F2x;
					force[j * n + (i + 1)].y += F2y;
					force[j * n + (i + 1)].z += F2z;

					force[(j + 1) * n + i].x -= F4x;
					force[(j + 1) * n + i].y -= F4y;
					force[(j + 1) * n + i].z -= F4z;

					force[(j + 1) * n + i].x -= F2x;
					force[(j + 1) * n + i].y -= F2y;
					force[(j + 1) * n + i].z -= F2z;
				}
			});

		for (std::size_t i = 0; i < n - 1; i++)
		{
			float F4x, F4y, F4z;
			float diffx;
			float diffy;
			float diffz;
			float velDiffx;
			float velDiffy;
			float velDiffz;
			float diffNorm;
			float k;

			//non-diag connection "up to down" horizontal component

			diffx = currPos[(m - 1) * n + i + 1].x - currPos[(m - 1) * n + i].x;
			diffy = currPos[(m - 1) * n + i + 1].y - currPos[(m - 1) * n + i].y;
			diffz = currPos[(m - 1) * n + i + 1].z - currPos[(m - 1) * n + i].z;
			diffNorm = sqrt(diffx * diffx + diffy * diffy + diffz * diffz);
			k = shortSpring * (diffNorm - dx) * (1 / diffNorm);
			//now force due to damper
			velDiffx = velocity[(m - 1) * n + i + 1].x - velocity[(m - 1) * n + i].x;
			velDiffy = velocity[(m - 1) * n + i + 1].y - velocity[(m - 1) * n + i].y;
			velDiffz = velocity[(m - 1) * n + i + 1].z - velocity[(m - 1) * n + i].z;

			F4x = k * diffx + shortDamp * velDiffx;
			F4y = k * diffy + shortDamp * velDiffy;
			F4z = k * diffz + shortDamp * velDiffz;


			force[(m - 1) * n + i + 1].x -= F4x;
			force[(m - 1) * n + i + 1].y -= F4y;
			force[(m - 1) * n + i + 1].z -= F4z;


			force[(m - 1) * n + i].x += F4x;
			force[(m - 1) * n + i].y += F4y;
			force[(m - 1) * n + i].z += F4z;


		}

		for (std::size_t j = 0; j < m - 1; j++)
		{
			float F4x, F4y, F4z;
			float diffx;
			float diffy;
			float diffz;
			float velDiffx;
			float velDiffy;
			float velDiffz;
			float diffNorm;
			float k;

			//non-diag connection "up to down" horizontal component

			diffx = currPos[(j + 1) * n + n - 1].x - currPos[j * n + n - 1].x;
			diffy = currPos[(j + 1) * n + n - 1].y - currPos[j * n + n - 1].y;
			diffz = currPos[(j + 1) * n + n - 1].z - currPos[j * n + n - 1].z;
			diffNorm = sqrt(diffx * diffx + diffy * diffy + diffz * diffz);
			k = shortSpring * (diffNorm - dx) * (1 / diffNorm);
			//now force due to damper
			velDiffx = velocity[(j + 1) * n + n - 1].x - velocity[j * n + n - 1].x;
			velDiffy = velocity[(j + 1) * n + n - 1].y - velocity[j * n + n - 1].y;
			velDiffz = velocity[(j + 1) * n + n - 1].z - velocity[j * n + n - 1].z;

			F4x = k * diffx + shortDamp * velDiffx;
			F4y = k * diffy + shortDamp * velDiffy;
			F4z = k * diffz + shortDamp * velDiffz;


			force[j * n + n - 1].x += F4x;
			force[j * n + n - 1].y += F4y;
			force[j * n + n - 1].z += F4z;


			force[(j + 1) * n + n - 1].x -= F4x;
			force[(j + 1) * n + n - 1].y -= F4y;
			force[(j + 1) * n + n - 1].z -= F4z;

		}

		//update the position's of the elements and velocities
		concurrency::parallel_for(0, int(m), [this, &n](int j)
			{
				for (std::size_t i = 1; i < n; ++i)
				{

					prevPos[j * numRows + i].x = currPos[j * n + i].x
						+ velocity[j * n + i].x * dt
						+ force[j * n + i].x * 0.5 * (1 / mass) * dt * dt;

					prevPos[j * numRows + i].y = currPos[j * n + i].y
						+ velocity[j * n + i].y * dt
						+ force[j * n + i].y * 0.5 * (1 / mass) * dt * dt;

					prevPos[j * numRows + i].z = currPos[j * n + i].z
						+ velocity[j * n + i].z * dt
						+ force[j * n + i].z * 0.5 * (1 / mass) * dt * dt;

					velocity[j * n + i].x = (prevPos[j * n + i].x - currPos[j * n + i].x) * (1 / dt);
					velocity[j * n + i].y = (prevPos[j * n + i].y - currPos[j * n + i].y) * (1 / dt);
					velocity[j * n + i].z = (prevPos[j * n + i].z - currPos[j * n + i].z) * (1 / dt);

				}
			});






		concurrency::parallel_for(0, int(m - 1), [this, &n](int j)
			{
				for (std::size_t i = 0; i < n - 1; ++i)
				{
					tangents[j * n + i].x = currPos[(j + 1) * n + i].x - currPos[j * n + i].x;
					tangents[j * n + i].y = currPos[(j + 1) * n + i].y - currPos[j * n + i].y;
					tangents[j * n + i].z = currPos[(j + 1) * n + i].z - currPos[j * n + i].z;
					MathHelper::Normalize(tangents[j * n + i]);

					bitangents[j * n + i].x = currPos[j * n + i + 1].x - currPos[j * n + i].x;
					bitangents[j * n + i].y = currPos[j * n + i + 1].y - currPos[j * n + i].y;
					bitangents[j * n + i].z = currPos[j * n + i + 1].z - currPos[j * n + i].z;
					MathHelper::Normalize(bitangents[j * n + i]);

					normals[j * n + i] = MathHelper::XMVectorf3Cross(bitangents[j * n + i], tangents[j * n + i]);

				}
			});


		for (std::size_t j = 0; j < m; j++)
		{
			normals[j * n + n - 1] = normals[j * n + n - 2];

		}
		for (std::size_t i = 0; i < n; i++)
		{
			normals[(m - 1) * n + i] = normals[(m - 2) * n + i];

		}


		std::swap(prevPos, currPos);

		t = 0.0f;

	}
}
