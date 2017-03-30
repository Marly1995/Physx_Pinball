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
	
	// some static variables for game state
	static bool playing = false;
	static int score = 0;
	static int extraLife = 0;
	static int lives = 0;

	struct FilterGroup
	{
		enum Enum
		{
			PLAYER		= (1 << 0),
			SCORE1		= (1 << 1),
			SCORE2		= (1 << 2),
			SCORE3		= (2 << 3),
			SCORE4		= (3 << 4),
			SCORE5		= (4 << 5)
			//add more if you need
		};
	};

	// adjusted trampoline class 
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
			// prevent bottom half from moving
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
		// vars for trigger handling
		bool closeflap = false;
		bool death = false;

		///Method called when the contact with the trigger object is detected.
		virtual void onTrigger(PxTriggerPair* pairs, PxU32 count) 
		{
			//you can read the trigger information here
			for (PxU32 i = 0; i < count; i++)
			{
				//filter out contact with the planes
				if (pairs[i].otherShape->getGeometryType() != PxGeometryType::ePLANE)
				{
					// check if the ball leaves the launch pad
					if(pairs[i].triggerShape->getName() == "flap" &&
						pairs[i].otherShape->getName() == "ball")
					{
						closeflap = true;
					}
					// check if the ball hits the floor
					if (pairs[i].triggerShape->getName() == "death" &&
						pairs[i].otherShape->getName() == "ball")
					{
						death = true;
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
				// collisions set up for different scores
				switch (pairs[i].shapes[0]->getSimulationFilterData().word0)
				{
				case FilterGroup::PLAYER:
					score += 100;
					break;
				case FilterGroup::SCORE1:
					score += 1000;
					break;
				case FilterGroup::SCORE2:
					score += 500;
					break;
				case FilterGroup::SCORE3:
					score += 200;
					break;
				case FilterGroup::SCORE4:
					score += 100;
					break;
				case FilterGroup::SCORE5:
					score += 50;
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
		Box* base, *roof, *box1, *box2, *box3, *box4, *box5;;
		Walls *walls;

		Sphere* ball;

		Trampoline *plunger;
		DistanceJoint *spring;
		bool pullSpring = false;
		float springStr = 0.0f;

		Wedge *padL, *padR, *padL2;
		RevoluteJoint *LPjoint, *LPjoint2, *RPjoint;

		Hexagon *spinner1, *spinner2;
		RevoluteJoint *spinnerJoint1, *spinnerJoint2;

		Dimond *topL, *topR;
		RevoluteJoint *TLjoint, *TRjoint;

		Box *flap;
		RevoluteJoint *flapjoint;

		Capsule *left, *top, *right;

		StaticBox * flapTrigger, *deathTrigger;
		
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
			lives = 3;
			score = 0;

			SetVisualisation();			
			CreateMaterial(0.94, 0.40, 0.1);
			CreateMaterial(1.10, 0.15, 0.2);
			CreateMaterial(0.25, 0.10, 0.4);
			CreateMaterial(0.78, 0.42, 0.2);
			CreateMaterial(0.04, 0.04, 0.1);
			CreateMaterial(0.38, 0.20, 0.4);
			CreateMaterial(0.10, 0.03, 0.1);
			CreateMaterial(0.10, 0.08, 0.2);
			// pinger using high cr
			CreateMaterial(0.25, 0.10, 4.0);

			// angle to tilt game objcet at
			float tableAngle = PxPi / 6;

			my_callback = new MySimulationEventCallback();
			px_scene->setSimulationEventCallback(my_callback);
			// allow ccd on objects that need it
			px_scene->setFlag(PxSceneFlag::eENABLE_CCD, true);

			// plane
			plane = new Plane();
			plane->Color(PxVec3(210.f/255.f,210.f/255.f,210.f/255.f));
			plane->Material(GetMaterial(2));
			Add(plane);

			// base for game board
			base = new Box(PxTransform(PxVec3(0.0f, 12.0f, 2.0f), PxQuat(tableAngle, PxVec3(0.0f, 0.0f, 1.0f))), PxVec3(20.0f, 0.5f, 13.0f));
			base->Color(color_palette[0]);
			base->Material(GetMaterial(6));
			base->SetKinematic(true);
			Add(base);

			// roof to prevent ball from flying out
			roof = new Box(PxTransform(PxVec3(-2.0f, 18.0f, 2.0f), PxQuat(tableAngle, PxVec3(0.0f, 0.0f, 1.0f))), PxVec3(20.0f, 0.5f, 13.0f));
			roof->Color(color_palette[0]);
			roof->Material(GetMaterial(6));
			roof->SetKinematic(true);
			roof->GetShape()->setFlag(PxShapeFlag::eVISUALIZATION, false);
			Add(roof);

			// BALL
			ball = new Sphere(PxTransform(PxVec3(-8.0f,12.0f, 12.0f), PxQuat(PxIdentity)), 0.3f);
			ball->Material(GetMaterial(3));
			ball->Color(color_palette[4]);
			ball->GetShape()->setName("ball");
			Add(ball);			
			// allow ccd for ball as it travels vert fast
			ball->Get()->isRigidBody()->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_CCD, true);
			// set up the ball to filter different collisions with various scoring objects
			ball->SetupFiltering(FilterGroup::PLAYER, FilterGroup::SCORE1 | FilterGroup::SCORE2 | FilterGroup::SCORE3 | FilterGroup::SCORE4 | FilterGroup::SCORE5);

			// PADDLES
			{
			// left paddle
			padL = new Wedge(4.0f, 1.5f, 1.f, PxTransform(PxVec3(-16.0f, 4.0f, -5.0f), PxQuat(tableAngle, PxVec3(0.0f, 0.0f, 1.0f))), 1.0f);
			padL->mesh->GetShape()->setLocalPose(PxTransform(PxVec3(0.0f, 0.0f, 0.0f), PxQuat(PxPi / 2, PxVec3(1.0f, 0.0f, 0.0f))));
			padL->mesh->Color(color_palette[2]);
			padL->mesh->SetKinematic(false);
			// paddles have ccd as they move fast to hit ball
			padL->mesh->Get()->isRigidBody()->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_CCD, true);
			Add(padL->mesh);
			// set up revolute joint for paddle motion and limit it to realistic constraints
			LPjoint = new RevoluteJoint(base, PxTransform(PxVec3(-16.f, 1.f, -6.f), PxQuat(PxPi/2, PxVec3(0.f, 0.f, 1.f))), padL->mesh, PxTransform(PxVec3(0.5f, 0.f, 0.5f), PxQuat(PxPi / 2, PxVec3(0.0f, 0.0f, 1.0f))));
			LPjoint->SetLimits(-PxPi/6, PxPi/6);
			padL->mesh->SetupFiltering(FilterGroup::SCORE5, FilterGroup::PLAYER);

			padL2 = new Wedge(4.0f, 1.5f, 1.f, PxTransform(PxVec3(-16.0f, 4.0f, -5.0f), PxQuat(tableAngle, PxVec3(0.0f, 0.0f, 1.0f))), 1.0f);
			padL2->mesh->GetShape()->setLocalPose(PxTransform(PxVec3(0.0f, 0.0f, 0.0f), PxQuat(PxPi / 2, PxVec3(1.0f, 0.0f, 0.0f))));
			padL2->mesh->Color(color_palette[2]);
			padL2->mesh->SetKinematic(false);
			Add(padL2->mesh);
			LPjoint2 = new RevoluteJoint(base, PxTransform(PxVec3(10.f, 1.f, -10.f), PxQuat(PxPi / 2, PxVec3(0.f, 0.f, 1.f))), padL2->mesh, PxTransform(PxVec3(0.5f, 0.f, 0.5f), PxQuat(PxPi / 2, PxVec3(0.0f, 0.0f, 1.0f))));
			LPjoint2->SetLimits(-PxPi/2, -PxPi/4);
			padL2->mesh->SetupFiltering(FilterGroup::SCORE4, FilterGroup::PLAYER);

			// actor 5 right paddle
			padR = new Wedge(4.0f, 1.5f, 1.f, PxTransform(PxVec3(-16.0f, 4.0f, 4.0f), PxQuat(tableAngle, PxVec3(0.0f, 0.0f, 1.0f))));
			padR->mesh->GetShape()->setLocalPose(PxTransform(PxVec3(0.0f, 0.0f, 0.0f), PxQuat(-PxPi / 2, PxVec3(1.0f, 0.0f, 0.0f))));
			padR->mesh->Color(color_palette[2]);
			padR->mesh->SetKinematic(false);
			padR->mesh->Get()->isRigidBody()->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_CCD, true);
			Add(padR->mesh);
			RPjoint = new RevoluteJoint(base, PxTransform(PxVec3(-16.f, 0.5f, 3.f), PxQuat(PxPi / 2, PxVec3(0.f, 0.f, 1.f))), padR->mesh, PxTransform(PxVec3(0.5f, 0.f, -0.5f), PxQuat(PxPi / 2, PxVec3(0.0f, 0.0f, 1.0f))));
			RPjoint->SetLimits(-PxPi/6, PxPi / 6);
			RPjoint->DriveVelocity(10.0f);
			padR->mesh->SetupFiltering(FilterGroup::SCORE5, FilterGroup::PLAYER);
			}

			// PLUNGER
			plunger = new Trampoline(PxVec3(.5f, 0.5f, 0.5f), 100.0f, 10.0f, PxTransform(PxVec3(-12.0f, 6.0f, 12.0f), PxQuat(-tableAngle*2, PxVec3(0.0f, 0.0f, 1.0f))), PxTransform(PxVec3(-12.0f, 6.0f, 12.0f), PxQuat(-tableAngle*2, PxVec3(0.0f, 0.0f, 1.0f))));
			plunger->AddToScene(this);

			// SPINNERS AND PINGERS
			{
			spinner1 = new Hexagon(.5f, 2.0f, PxTransform(PxVec3(10.0f, 16.0f, -4.0f), PxQuat(tableAngle, PxVec3(0.0f, 0.0f, 1.0f))));
			Add(spinner1->mesh);
			spinnerJoint1 = new RevoluteJoint(base, PxTransform(PxVec3(6.f, 0.5f, -4.5f), PxQuat(PxPi / 2, PxVec3(0.f, 0.f, 1.f))), spinner1->mesh, PxTransform(PxVec3(0.0f, 0.f, 0.0f), PxQuat(PxPi / 2, PxVec3(0.0f, 0.0f, 1.0f))));	
			spinner1->mesh->SetupFiltering(FilterGroup::SCORE2, FilterGroup::PLAYER);

			spinner2 = new Hexagon(.5f, 2.0f, PxTransform(PxVec3(10.0f, 16.0f, 4.0f), PxQuat(tableAngle, PxVec3(0.0f, 0.0f, 1.0f))));
			Add(spinner2->mesh);
			spinnerJoint2 = new RevoluteJoint(base, PxTransform(PxVec3(6.f, 0.5f, 1.5f), PxQuat(PxPi / 2, PxVec3(0.f, 0.f, 1.f))), spinner2->mesh, PxTransform(PxVec3(0.0f, 0.f, 0.0f), PxQuat(PxPi / 2, PxVec3(0.0f, 0.0f, 1.0f))));
			spinner2->mesh->SetupFiltering(FilterGroup::SCORE2, FilterGroup::PLAYER);

			top = new Capsule(PxTransform(PxVec3(13.0f, 20.0f, 0.5f), PxQuat(tableAngle+PxHalfPi, PxVec3(0.0f, 0.0f, 1.0f))), PxVec2(1.0f, 1.0f));
			top->SetKinematic(true);
			top->Material(GetMaterial(9));
			Add(top);
			top->SetupFiltering(FilterGroup::SCORE1, FilterGroup::PLAYER);

			left = new Capsule(PxTransform(PxVec3(9.0f, 17.5f, 3.5f), PxQuat(tableAngle + PxHalfPi, PxVec3(0.0f, 0.0f, 1.0f))), PxVec2(1.0f, 1.0f));
			left->SetKinematic(true);
			left->Material(GetMaterial(9));
			Add(left);
			left->SetupFiltering(FilterGroup::SCORE1, FilterGroup::PLAYER);

			right = new Capsule(PxTransform(PxVec3(9.0f, 17.5f, -2.5f), PxQuat(tableAngle + PxHalfPi, PxVec3(0.0f, 0.0f, 1.0f))), PxVec2(1.0f, 1.0f));
			right->SetKinematic(true);
			right->Material(GetMaterial(9));
			Add(right);
			right->SetupFiltering(FilterGroup::SCORE1, FilterGroup::PLAYER);

			topR = new Dimond(2.0f, 1.0f, 1.0f, PxTransform(PxVec3(8.0f, 14.0f, 6.0f), PxQuat(tableAngle, PxVec3(0.0f, 0.0f, 1.0f))), 0.5f);
			topR->mesh->GetShape()->setLocalPose(PxTransform(PxVec3(0.0f, 0.0f, 0.0f), PxQuat(PxPi / 2, PxVec3(1.0f, 0.0f, 0.0f))));
			Add(topR->mesh);
			TRjoint = new RevoluteJoint(base, PxTransform(PxVec3(2.0f, 0.0f, 4.5f), PxQuat(PxPi / 2, PxVec3(0.f, 0.f, 1.f))), topR->mesh, PxTransform(PxVec3(0.5f, -1.f, 0.0f), PxQuat(PxPi / 2, PxVec3(0.0f, 0.0f, 1.0f))));
			topR->mesh->SetupFiltering(FilterGroup::SCORE3, FilterGroup::PLAYER);

			topL = new Dimond(2.0f, 1.0f, 1.0f, PxTransform(PxVec3(8.0f, 14.0f, -6.0f), PxQuat(tableAngle, PxVec3(0.0f, 0.0f, 1.0f))), 0.5f);
			topL->mesh->GetShape()->setLocalPose(PxTransform(PxVec3(0.0f, 0.0f, 0.0f), PxQuat(PxPi / 2, PxVec3(1.0f, 0.0f, 0.0f))));
			Add(topL->mesh);
			TLjoint = new RevoluteJoint(base, PxTransform(PxVec3(2.0f, 1.0f, -7.5f), PxQuat(PxPi / 2, PxVec3(0.f, 0.f, 1.f))), topL->mesh, PxTransform(PxVec3(0.5f, 0.f, 0.0f), PxQuat(PxPi / 2, PxVec3(0.0f, 0.0f, 1.0f))));
			topL->mesh->SetupFiltering(FilterGroup::SCORE3, FilterGroup::PLAYER);
			}

			// WALLS AND BOARD GEOMETRY
			{
			walls = new Walls(PxTransform(PxVec3(PxIdentity), PxQuat(PxPi / 6, PxVec3(0.0f, 0.0f, 1.0f))), PxVec3(20.0f, 2.0f, 0.5f));
			walls->GetShape(0)->setLocalPose(PxTransform(PxVec3(4.0f, 12.0f, 13.0f), PxQuat(PxIdentity)));
			walls->GetShape(1)->setLocalPose(PxTransform(PxVec3(4.0f, 12.0f, -10.0f), PxQuat(PxIdentity)));
			walls->GetShape(2)->setLocalPose(PxTransform(PxVec3(-14.0f, 12.0f, 1.0f), PxQuat(PxIdentity)));
			walls->GetShape(3)->setLocalPose(PxTransform(PxVec3(24.0f, 12.0f, 1.0f), PxQuat(PxIdentity)));
			walls->SetKinematic(true);
			Add(walls);

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
			}

			// FLAP TO PREVENT BALL RE ENTERING PLUNGER
			flap = new Box(PxTransform(PxVec3(8.0f, 17.0f, 11.0f), PxQuat(tableAngle, PxVec3(0.0f, 0.0f, 1.0f))), PxVec3(1.5f, 1.0f, 0.2f), 0.1f);
			flapjoint = new RevoluteJoint(base, PxTransform(PxVec3(8.5f, 1.5f, 8.f), PxQuat(PxPi / 2, PxVec3(0.f, 0.f, 1.f))), flap, PxTransform(PxVec3(1.3f, 0.f, 0.0f), PxQuat(PxPi / 2, PxVec3(0.0f, 0.0f, 1.0f))));
			Add(flap);

			// trigger to activate flap
			flapTrigger = new StaticBox(PxTransform(PxVec3(8.0f, 17.0f, 12.0f), PxQuat(tableAngle, PxVec3(0.0f, 0.0f, 1.0f))), PxVec3(1.0f, 1.0f, 1.0f), 0.1f);
			flapTrigger->SetTrigger(true);
			flapTrigger->GetShape()->setFlag(PxShapeFlag::eVISUALIZATION, false);
			flapTrigger->GetShape()->setName("flap");
			Add(flapTrigger);

			// rtigger to detect when the ball hits the floor
			deathTrigger = new StaticBox(PxTransform(PxVec3(-17.0f, 4.0f, 0.0f), PxQuat(tableAngle, PxVec3(0.0f, 0.0f, 1.0f))), PxVec3(0.5f, 1.0f, 10.0f), 0.1f);
			deathTrigger->SetTrigger(true);
			deathTrigger->GetShape()->setFlag(PxShapeFlag::eVISUALIZATION, false);
			deathTrigger->GetShape()->setName("death");
			Add(deathTrigger);


			// set some joints to right location
			LPjoint->DriveVelocity(-10.0f);
			flapjoint->DriveVelocity(1.0f);
		}

		// reset the ball when it hits floor
		virtual void resetBall()
		{
			ball = new Sphere(PxTransform(PxVec3(-8.0f, 12.0f, 12.0f), PxQuat(PxIdentity)), 0.3f);
			ball->Material(GetMaterial(3));
			ball->Color(color_palette[4]);
			ball->GetShape()->setName("ball");
			Add(ball);
			ball->Get()->isRigidBody()->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_CCD, true);
			ball->SetupFiltering(FilterGroup::PLAYER, FilterGroup::SCORE1 | FilterGroup::SCORE2 | FilterGroup::SCORE3 | FilterGroup::SCORE4 | FilterGroup::SCORE5);
			flapjoint->DriveVelocity(10.0f);
		}

		// spawn a new ball atthe top of the board
		virtual void multiBall()
		{
			ball = new Sphere(PxTransform(PxVec3(10.0f, 20.0f, -8.0f), PxQuat(PxIdentity)), 0.3f);
			ball->Material(GetMaterial(3));
			ball->Color(color_palette[4]);
			ball->GetShape()->setName("ball");
			Add(ball);
			ball->Get()->isRigidBody()->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_CCD, true);
			ball->SetupFiltering(FilterGroup::PLAYER, FilterGroup::SCORE1 | FilterGroup::SCORE2 | FilterGroup::SCORE3 | FilterGroup::SCORE4 | FilterGroup::SCORE5);
		}

		//Custom udpate function
		virtual void CustomUpdate() 
		{
			if(playing == true)
			{
				//score++;
				extraLife++;
			}
			
			spinnerJoint1->DriveVelocity(20.0f);
			spinnerJoint2->DriveVelocity(-20.0f);
			TRjoint->DriveVelocity(-10.0f);
			TLjoint->DriveVelocity(10.0f);

			if (pullSpring)
			{
				springStr += 4.0f;
				if (springStr >= 400.0f) { springStr = 400.0f; }		
			}

			if (my_callback->closeflap == true)
			{
				my_callback->closeflap = false;
				flapjoint->DriveVelocity(-10.0f);
				playing = true;
			}

			if (my_callback->death == true)
			{
				my_callback->death = false;
				resetBall();
				lives--;
				playing = false;
			}

			if(lives == 0)
			{
				Reset();
			}
			if(extraLife >= 10000)
			{
				multiBall();
				extraLife = 0;
				lives++;
			}

		}

		virtual int GetScore()
		{
			return score;
		}

		virtual int GetLives()
		{
			return lives;
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
			LPjoint2->DriveVelocity(10.0f);
		}
		void KeyReleaseE()
		{
			LPjoint->DriveVelocity(-10.0f);
			LPjoint2->DriveVelocity(-10.0f);
		}

		void KeyPressL()
		{
			pullSpring = true;
		}

		void KeyReleaseL()
		{
			pullSpring = false;
			this->SelectActor(7);
			this->GetSelectedActor()->addForce(PxVec3(2.0f, 1.0f, 0.0f) * springStr);			
			springStr = 0.0f;

		}
	};
}
