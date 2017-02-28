#pragma once

#include "PhysicsEngine.h"
#include "BasicActors.h"
#include <iostream>
#include <iomanip>

namespace PhysicsEngine
{
	class CompoundShape : public DynamicActor
	{
	public:
		CompoundShape(const PxTransform& pose = PxTransform(PxIdentity), PxVec3 dimensions = PxVec3(0.5f, 0.5f, 0.5f), PxReal density = 1.0f)
			: DynamicActor(pose)
		{
			CreateShape(PxBoxGeometry(dimensions), density);
			CreateShape(PxBoxGeometry(dimensions), density);
		}
	};

	class CapsuleWheel : public DynamicActor
	{
	public:
		CapsuleWheel(const PxTransform& pose = PxTransform(PxIdentity), PxVec2 dimensions = PxVec2(0.5f, 1.0f), PxReal density = 1.0f)
			: DynamicActor(pose)
		{
			CreateShape(PxCapsuleGeometry(dimensions.x, dimensions.y), density);
			CreateShape(PxCapsuleGeometry(dimensions.x, dimensions.y), density);
			CreateShape(PxCapsuleGeometry(dimensions.x, dimensions.y), density);
			CreateShape(PxCapsuleGeometry(dimensions.x, dimensions.y), density);
			CreateShape(PxCapsuleGeometry(dimensions.x, dimensions.y), density);
			CreateShape(PxCapsuleGeometry(dimensions.x, dimensions.y), density);
			CreateShape(PxCapsuleGeometry(dimensions.x, dimensions.y), density);
			CreateShape(PxCapsuleGeometry(dimensions.x, dimensions.y), density);
		}
	};

	class Wedge
	{
	public:
		vector<PxVec3> verts = { PxVec3(0,1,0), PxVec3(1,0,0), PxVec3(-1,0,0), PxVec3(0,0,1), PxVec3(0,0,-1) };
		ConvexMesh* mesh = new ConvexMesh(vector<PxVec3>(begin(verts), end(verts)));

		Wedge(float h, float w, float l, PxTransform pose = PxTransform(PxIdentity), PxReal density = 1.0f)
		{
			verts[0] *= h;
			verts[1] *= w;
			verts[2] *= w;
			verts[3] *= l;
			verts[4] *= l;
			mesh = new ConvexMesh(vector<PxVec3>(begin(verts), end(verts)), pose, density);
		}
	};

	class Hexagon
	{
	public:
		vector<PxVec3> verts{ PxVec3(0,1,0), PxVec3(0,1,0), PxVec3(-1,0,0), PxVec3(0,0,1), PxVec3(0,0,-1), PxVec3(0,0,-1) };
		ConvexMesh* mesh = new ConvexMesh(vector<PxVec3>(begin(verts), end(verts)));

		Hexagon(float h, float edge, PxTransform pose = PxTransform(PxIdentity), PxReal density = 1.0f)
		{
			float theta = 0.0f;

			for (int i = 0; i < 3; i++)
			{
				theta = i * 2.0f * PxPi / 6;
				verts[i] = PxVec3(edge * cosf(theta), 0.0f, edge * sinf(theta));
			}
			for (int i = 0; i < 3; i++)
			{
				verts.push_back(PxVec3(verts[i].x, verts[i].y + h, verts[i].z));
			}
			mesh = new ConvexMesh(vector<PxVec3>(begin(verts), end(verts)), pose, density);
		}
	};
}