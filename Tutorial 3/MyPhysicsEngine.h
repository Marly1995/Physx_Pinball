#pragma once

#include "BasicActors.h"
#include "CompoundActors.h"
#include <iostream>
#include <iomanip>

namespace PhysicsEngine
{
	using namespace std;

	//a list of colours: Circus Palette
	static const PxVec3 color_palette[] = {PxVec3(46.f/255.f,9.f/255.f,39.f/255.f),PxVec3(217.f/255.f,0.f/255.f,0.f/255.f),
		PxVec3(255.f/255.f,45.f/255.f,0.f/255.f),PxVec3(255.f/255.f,140.f/255.f,54.f/255.f),PxVec3(4.f/255.f,117.f/255.f,111.f/255.f)};

	static int score = 0;
	//pyramid vertices
	static PxVec3 pyramid_verts[] = {PxVec3(0,1,0), PxVec3(1,0,0), PxVec3(-1,0,0), PxVec3(0,0,1), PxVec3(0,0,-1)};
	//pyramid triangles: a list of three vertices for each triangle e.g. the first triangle consists of vertices 1, 4 and 0
	//vertices have to be specified in a counter-clockwise order to assure the correct shading in rendering
	static PxU32 pyramid_trigs[] = {1, 4, 0, 3, 1, 0, 2, 3, 0, 4, 2, 0, 3, 2, 1, 2, 4, 1};

	class Pyramid : public ConvexMesh
	{
	public:
		Pyramid(PxTransform pose=PxTransform(PxIdentity), PxReal density=1.f) :
			ConvexMesh(vector<PxVec3>(begin(pyramid_verts),end(pyramid_verts)), pose, density)
		{
		}
	};

	class PyramidStatic : public TriangleMesh
	{
	public:
		PyramidStatic(PxTransform pose=PxTransform(PxIdentity)) :
			TriangleMesh(vector<PxVec3>(begin(pyramid_verts),end(pyramid_verts)), vector<PxU32>(begin(pyramid_trigs),end(pyramid_trigs)), pose)
		{
		}
	};

	struct FilterGroup
	{
		enum Enum
		{
			ACTOR0		= (1 << 0),
			ACTOR1		= (1 << 1),
			ACTOR2		= (1 << 2)
			//add more if you need
		};
	};

	///An example class showing the use of springs (distance joints).
	class Trampoline
	{
	public:
		vector<DistanceJoint*> springs;
		Box *bottom, *top;

		Trampoline(const PxVec3& dimensions=PxVec3(1.f,1.f,1.f), PxReal stiffness=1.f, PxReal damping=1.f, PxTransform botPos = PxTransform(PxIdentity), PxTransform topPos = PxTransform(PxIdentity))
		{
			PxReal thickness = 0.1f;
			bottom = new Box(PxTransform(PxVec3(botPos.p.x, topPos.p.y + thickness, botPos.p.z), PxQuat(botPos.q)), PxVec3(dimensions.x, thickness, dimensions.z));
			top = new Box(PxTransform(PxVec3(topPos.p.x, topPos.p.y + dimensions.y + thickness, topPos.p.z), PxQuat(topPos.q)), PxVec3(dimensions.x, thickness, dimensions.z));
			springs.resize(4);
			springs[0] = new DistanceJoint(bottom, PxTransform(PxVec3(dimensions.x,thickness,dimensions.z)), top, PxTransform(PxVec3(dimensions.x,-dimensions.y,dimensions.z)));
			springs[1] = new DistanceJoint(bottom, PxTransform(PxVec3(dimensions.x,thickness,-dimensions.z)), top, PxTransform(PxVec3(dimensions.x,-dimensions.y,-dimensions.z)));
			springs[2] = new DistanceJoint(bottom, PxTransform(PxVec3(-dimensions.x,thickness,dimensions.z)), top, PxTransform(PxVec3(-dimensions.x,-dimensions.y,dimensions.z)));
			springs[3] = new DistanceJoint(bottom, PxTransform(PxVec3(-dimensions.x,thickness,-dimensions.z)), top, PxTransform(PxVec3(-dimensions.x,-dimensions.y,-dimensions.z)));

			for (unsigned int i = 0; i < springs.size(); i++)
			{
				springs[i]->Stiffness(stiffness);
				springs[i]->Damping(damping);
			}
			bottom->SetKinematic(true);
		}

		void AddToScene(Scene* scene)
		{
			scene->Add(bottom);
			scene->Add(top);
		}

		~Trampoline()
		{
			for (unsigned int i = 0; i < springs.size(); i++)
				delete springs[i];
		}
	};

	///A customised collision class, implemneting various callbacks
	class MySimulationEventCallback : public PxSimulationEventCallback
	{
	public:
		//an example variable that will be checked in the main simulation loop
		bool trigger;

		MySimulationEventCallback() : trigger(false) {}

		///Method called when the contact with the trigger object is detected.
		virtual void onTrigger(PxTriggerPair* pairs, PxU32 count) 
		{
			//you can read the trigger information here
			for (PxU32 i = 0; i < count; i++)
			{
				//filter out contact with the planes
				if (pairs[i].otherShape->getGeometryType() != PxGeometryType::ePLANE)
				{
					//check if eNOTIFY_TOUCH_FOUND trigger
					if (pairs[i].status & PxPairFlag::eNOTIFY_TOUCH_FOUND)
					{
						cerr << "onTrigger::eNOTIFY_TOUCH_FOUND" << endl;
						trigger = true;
					}
					//check if eNOTIFY_TOUCH_LOST trigger
					if (pairs[i].status & PxPairFlag::eNOTIFY_TOUCH_LOST)
					{
						cerr << "onTrigger::eNOTIFY_TOUCH_LOST" << endl;
						trigger = false;
					}
				}
			}
		}

		///Method called when the contact by the filter shader is detected.
		virtual void onContact(const PxContactPairHeader &pairHeader, const PxContactPair *pairs, PxU32 nbPairs) 
		{
			cerr << "Contact found between " << pairHeader.actors[0]->getName() << " " << pairHeader.actors[1]->getName() << endl;

			//check all pairs
			for (PxU32 i = 0; i < nbPairs; i++)
			{
				////check eNOTIFY_TOUCH_FOUND
				//if (pairs[i].events & PxPairFlag::eNOTIFY_TOUCH_FOUND)
				//{
				//	cerr << "onContact::eNOTIFY_TOUCH_FOUND" << endl;
				//}
				////check eNOTIFY_TOUCH_LOST
				//if (pairs[i].events & PxPairFlag::eNOTIFY_TOUCH_LOST)
				//{
				//	cerr << "onContact::eNOTIFY_TOUCH_LOST" << endl;
				//}
				switch (pairs[i].shapes[0]->getSimulationFilterData().word0)
				{
				case FilterGroup::ACTOR0:
					score += 10;
					break;
				case FilterGroup::ACTOR1:
					score += 10;
					break;
				case FilterGroup::ACTOR2:
					score += 5;
					break;
				}
			}
		}

		virtual void onConstraintBreak(PxConstraintInfo *constraints, PxU32 count) {}
		virtual void onWake(PxActor **actors, PxU32 count) {}
		virtual void onSleep(PxActor **actors, PxU32 count) {}
	};

	//A simple filter shader based on PxDefaultSimulationFilterShader - without group filtering
	static PxFilterFlags CustomFilterShader( PxFilterObjectAttributes attributes0,	PxFilterData filterData0,
		PxFilterObjectAttributes attributes1,	PxFilterData filterData1,
		PxPairFlags& pairFlags,	const void* constantBlock,	PxU32 constantBlockSize)
	{
		// let triggers through
		if(PxFilterObjectIsTrigger(attributes0) || PxFilterObjectIsTrigger(attributes1))
		{
			pairFlags = PxPairFlag::eTRIGGER_DEFAULT;
			return PxFilterFlags();
		}

		//enable continous collision detection
		pairFlags = PxPairFlag::eSOLVE_CONTACT;
		pairFlags |= PxPairFlag::eDETECT_DISCRETE_CONTACT;
		pairFlags |= PxPairFlag::eDETECT_CCD_CONTACT;
		
		
		//customise collision filtering here
		//e.g.

		// trigger the contact callback for pairs (A,B) where 
		// the filtermask of A contains the ID of B and vice versa.
		if((filterData0.word0 & filterData1.word1) && (filterData1.word0 & filterData0.word1))
		{
			//trigger onContact callback for this pair of objects
			pairFlags |= PxPairFlag::eNOTIFY_TOUCH_FOUND;
			pairFlags |= PxPairFlag::eNOTIFY_TOUCH_LOST;
//			pairFlags |= PxPairFlag::eNOTIFY_CONTACT_POINTS;
		}

		return PxFilterFlags();
	};

	///Custom scene class
	class MyScene : public Scene
	{
		MySimulationEventCallback* my_callback;
		Plane* plane;
		Box* base;
		Sphere* ball;
		Walls *walls;

		Trampoline *plunger;
		DistanceJoint *spring;
		bool pullSpring = false;
		float springStr = 0.0f;

		Wedge *padL, *padR;
		RevoluteJoint *LPjoint, *RPjoint;

		Hexagon *spinner;
		RevoluteJoint *spinnerJoint;

		Dimond *topL, *topR;
		RevoluteJoint *TLjoint, *TRjoint;

		Box *box1, *box2, *box3, *box4, *box5;
		Box *flap;
		
	public:
		//specify your custom filter shader here
		//PxDefaultSimulationFilterShader by default
		MyScene() : Scene(CustomFilterShader) {};

		///A custom scene class
		void SetVisualisation()
		{
			px_scene->setVisualizationParameter(PxVisualizationParameter::eSCALE, 1.0f);
			px_scene->setVisualizationParameter(PxVisualizationParameter::eCOLLISION_SHAPES, 1.0f);
			px_scene->setVisualizationParameter(PxVisualizationParameter::eJOINT_LOCAL_FRAMES, 1.0f);
			px_scene->setVisualizationParameter(PxVisualizationParameter::eJOINT_LIMITS, 1.0f);

		}

		//Custom scene initialisation
		virtual void CustomInit() 
		{
			SetVisualisation();			
			CreateMaterial(0.94, 0.40, 0.1);
			CreateMaterial(1.10, 0.15, 0.2);
			CreateMaterial(0.25, 0.10, 0.4);
			CreateMaterial(0.78, 0.42, 0.2);
			CreateMaterial(0.04, 0.04, 0.1);
			CreateMaterial(0.38, 0.20, 0.4);
			CreateMaterial(0.10, 0.03, 0.1);
			CreateMaterial(0.10, 0.08, 0.2);

			float tableAngle = PxPi / 6;

			///Initialise and set the customised event callback
			my_callback = new MySimulationEventCallback();
			px_scene->setSimulationEventCallback(my_callback);
			px_scene->setFlag(PxSceneFlag::eENABLE_CCD, true);

			// actor 1 plane
			plane = new Plane();
			plane->Color(PxVec3(210.f/255.f,210.f/255.f,210.f/255.f));
			plane->Material(GetMaterial(2));
			Add(plane);

			// actor 2 base
			base = new Box(PxTransform(PxVec3(0.0f, 12.0f, 2.0f), PxQuat(tableAngle, PxVec3(0.0f, 0.0f, 1.0f))), PxVec3(20.0f, 0.5f, 13.0f));
			base->Color(color_palette[0]);
			base->Material(GetMaterial(6));
			base->SetKinematic(true);
			Add(base);

			// actor 3 ball
			ball = new Sphere(PxTransform(PxVec3(-8.0f,12.0f, 12.0f), PxQuat(PxIdentity)), 0.3f);
			ball->Material(GetMaterial(3));
			ball->Color(color_palette[4]);
			Add(ball);			
			ball->Get()->isRigidBody()->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_CCD, true);
			ball->SetupFiltering(FilterGroup::ACTOR0, FilterGroup::ACTOR1 | FilterGroup::ACTOR2);

			// actor 4 left paddle
			padL = new Wedge(4.0f, 1.5f, 1.f, PxTransform(PxVec3(-16.0f, 4.0f, -5.0f), PxQuat(tableAngle, PxVec3(0.0f, 0.0f, 1.0f))), 1.0f);
			padL->mesh->GetShape()->setLocalPose(PxTransform(PxVec3(0.0f, 0.0f, 0.0f), PxQuat(PxPi / 2, PxVec3(1.0f, 0.0f, 0.0f))));
			padL->mesh->Color(color_palette[2]);
			padL->mesh->SetKinematic(false); 
			Add(padL->mesh);
			LPjoint = new RevoluteJoint(base, PxTransform(PxVec3(-16.f, 1.f, -6.f), PxQuat(PxPi/2, PxVec3(0.f, 0.f, 1.f))), padL->mesh, PxTransform(PxVec3(0.5f, 0.f, 0.5f), PxQuat(PxPi / 2, PxVec3(0.0f, 0.0f, 1.0f))));
			LPjoint->SetLimits(-PxPi/6, PxPi/6);
			padL->mesh->SetupFiltering(FilterGroup::ACTOR2, FilterGroup::ACTOR0);

			// actor 5 right paddle
			padR = new Wedge(4.0f, 1.5f, 1.f, PxTransform(PxVec3(-16.0f, 4.0f, 4.0f), PxQuat(tableAngle, PxVec3(0.0f, 0.0f, 1.0f))));
			padR->mesh->GetShape()->setLocalPose(PxTransform(PxVec3(0.0f, 0.0f, 0.0f), PxQuat(-PxPi / 2, PxVec3(1.0f, 0.0f, 0.0f))));
			padR->mesh->Color(color_palette[2]);
			padR->mesh->SetKinematic(false);
			Add(padR->mesh);
			RPjoint = new RevoluteJoint(base, PxTransform(PxVec3(-16.f, 0.5f, 3.f), PxQuat(PxPi / 2, PxVec3(0.f, 0.f, 1.f))), padR->mesh, PxTransform(PxVec3(0.5f, 0.f, -0.5f), PxQuat(PxPi / 2, PxVec3(0.0f, 0.0f, 1.0f))));
			RPjoint->SetLimits(-PxPi/6, PxPi / 6);
			RPjoint->DriveVelocity(10.0f);
			padR->mesh->SetupFiltering(FilterGroup::ACTOR2, FilterGroup::ACTOR0);

			// actor 6 and 7 bottom and top respective plunger components
			plunger = new Trampoline(PxVec3(.5f, 0.5f, 0.5f), 100.0f, 10.0f, PxTransform(PxVec3(-12.0f, 6.0f, 12.0f), PxQuat(-tableAngle*2, PxVec3(0.0f, 0.0f, 1.0f))), PxTransform(PxVec3(-12.0f, 6.0f, 12.0f), PxQuat(-tableAngle*2, PxVec3(0.0f, 0.0f, 1.0f))));
			plunger->AddToScene(this);

			walls = new Walls(PxTransform(PxVec3(PxIdentity), PxQuat(PxPi/6, PxVec3(0.0f, 0.0f, 1.0f))), PxVec3(20.0f, 2.0f, 0.5f));
			walls->GetShape(0)->setLocalPose(PxTransform(PxVec3(4.0f, 12.0f, 13.0f), PxQuat(PxIdentity)));
			walls->GetShape(1)->setLocalPose(PxTransform(PxVec3(4.0f, 12.0f, -10.0f), PxQuat(PxIdentity)));
			walls->GetShape(2)->setLocalPose(PxTransform(PxVec3(-14.0f, 12.0f, 1.0f), PxQuat(PxIdentity)));
			walls->GetShape(3)->setLocalPose(PxTransform(PxVec3(24.0f, 12.0f, 1.0f), PxQuat(PxIdentity)));
			walls->SetKinematic(true);
			Add(walls);

			spinner = new Hexagon(.5f, 2.0f, PxTransform(PxVec3(10.0f, 16.0f, 0.0f), PxQuat(tableAngle, PxVec3(0.0f, 0.0f, 1.0f))));
			Add(spinner->mesh);
			spinnerJoint = new RevoluteJoint(base, PxTransform(PxVec3(11.f, 0.5f, -1.5f), PxQuat(PxPi / 2, PxVec3(0.f, 0.f, 1.f))), spinner->mesh, PxTransform(PxVec3(0.0f, 0.f, 0.0f), PxQuat(PxPi / 2, PxVec3(0.0f, 0.0f, 1.0f))));	
			spinner->mesh->SetupFiltering(FilterGroup::ACTOR1, FilterGroup::ACTOR0);

			topR = new Dimond(2.0f, 1.0f, 1.0f, PxTransform(PxVec3(8.0f, 14.0f, 6.0f), PxQuat(tableAngle, PxVec3(0.0f, 0.0f, 1.0f))), 0.5f);
			topR->mesh->GetShape()->setLocalPose(PxTransform(PxVec3(0.0f, 0.0f, 0.0f), PxQuat(PxPi / 2, PxVec3(1.0f, 0.0f, 0.0f))));
			Add(topR->mesh);
			TRjoint = new RevoluteJoint(base, PxTransform(PxVec3(8.0f, 0.0f, 4.5f), PxQuat(PxPi / 2, PxVec3(0.f, 0.f, 1.f))), topR->mesh, PxTransform(PxVec3(0.5f, -1.f, 0.0f), PxQuat(PxPi / 2, PxVec3(0.0f, 0.0f, 1.0f))));

			topL = new Dimond(2.0f, 1.0f, 1.0f, PxTransform(PxVec3(8.0f, 14.0f, -6.0f), PxQuat(tableAngle, PxVec3(0.0f, 0.0f, 1.0f))), 0.5f);
			topL->mesh->GetShape()->setLocalPose(PxTransform(PxVec3(0.0f, 0.0f, 0.0f), PxQuat(PxPi / 2, PxVec3(1.0f, 0.0f, 0.0f))));
			Add(topL->mesh);
			TLjoint = new RevoluteJoint(base, PxTransform(PxVec3(8.0f, 1.0f, -7.5f), PxQuat(PxPi / 2, PxVec3(0.f, 0.f, 1.f))), topL->mesh, PxTransform(PxVec3(0.5f, 0.f, 0.0f), PxQuat(PxPi / 2, PxVec3(0.0f, 0.0f, 1.0f))));

			box1 = new Box(PxTransform(PxVec3(13.0f, 20.0f, -8.0f), PxQuat(tableAngle, PxVec3(0.0f, 0.0f, 1.0f))), PxVec3(4.0f, 1.0f, 0.5f));
			box1->GetShape()->setLocalPose(PxTransform(PxVec3(0.0f, 0.0f, 0.0f), PxQuat(-PxPi / 4, PxVec3(0.0f, 1.0f, 0.0f))));
			box1->SetKinematic(true);
			Add(box1);

			box2 = new Box(PxTransform(PxVec3(13.0f, 20.0f, 9.0f), PxQuat(tableAngle, PxVec3(0.0f, 0.0f, 1.0f))), PxVec3(6.0f, 1.0f, 0.5f));
			box2->GetShape()->setLocalPose(PxTransform(PxVec3(0.0f, 0.0f, 0.0f), PxQuat(PxPi / 4, PxVec3(0.0f, 1.0f, 0.0f))));
			box2->SetKinematic(true);
			Add(box2);

			box3 = new Box(PxTransform(PxVec3(-12.0f, 6.0f, -8.0f), PxQuat(tableAngle, PxVec3(0.0f, 0.0f, 1.0f))), PxVec3(3.0f, 1.0f, 0.5f));
			box3->GetShape()->setLocalPose(PxTransform(PxVec3(0.0f, 0.0f, 0.0f), PxQuat(PxPi / 3, PxVec3(0.0f, 1.0f, 0.0f))));
			box3->SetKinematic(true);
			Add(box3);

			box4 = new Box(PxTransform(PxVec3(-12.0f, 6.0f, 8.5f), PxQuat(tableAngle, PxVec3(0.0f, 0.0f, 1.0f))), PxVec3(2.5f, 1.0f, 0.5f));
			box4->GetShape()->setLocalPose(PxTransform(PxVec3(0.0f, 0.0f, 0.0f), PxQuat(-PxPi / 3, PxVec3(0.0f, 1.0f, 0.0f))));
			box4->SetKinematic(true);
			Add(box4);

			box5 = new Box(PxTransform(PxVec3(-5.0f, 10.0f, 11.0f), PxQuat(tableAngle, PxVec3(0.0f, 0.0f, 1.0f))), PxVec3(14.f, 1.0f, 0.5f));
			box5->SetKinematic(true);
			Add(box5);

			flap = new Box(PxTransform(PxVec3(8.0f, 17.0f, 11.0f), PxQuat(tableAngle, PxVec3(0.0f, 0.0f, 1.0f))), PxVec3(2.5f, 1.0f, 0.2f), 0.1f);
			
		}

		//Custom udpate function
		virtual void CustomUpdate() 
		{
			spinnerJoint->DriveVelocity(10.0f);

			if (pullSpring)
			{
				springStr += 4.0f;
				if (springStr >= 400.0f) { springStr = 400.0f; }
				//this->SelectActor(5);			
				//PxTransform current = this->GetSelectedActor()->getGlobalPose();
				//PxVec3 vec = PxVec3(current.p.x - 0.01f, current.p.y - 0.02f, current.p.z);
				//this->GetSelectedActor()->setGlobalPose(PxTransform(vec, current.q));				
			}
		}

		virtual int GetScore()
		{
			return score;
		}

		void KeyPressR()
		{
			RPjoint->DriveVelocity(-10.0f);
		}
		void KeyReleaseR()
		{
			RPjoint->DriveVelocity(10.0f);
		}	

		void KeyPressE()
		{
			LPjoint->DriveVelocity(10.0f);
		}
		void KeyReleaseE()
		{
			LPjoint->DriveVelocity(-10.0f);
		}

		void KeyPressL()
		{
			pullSpring = true;
			//plunger->top->SetKinematic(true);
		}

		void KeyReleaseL()
		{
			pullSpring = false;
			this->SelectActor(5);
			this->GetSelectedActor()->addForce(PxVec3(2.0f, 1.0f, 0.0f) * springStr);			
			springStr = 0.0f;
			//plunger->top->SetKinematic(false);

		}
	};
}
