#pragma once

#include "BasicActors.h"
#include <iostream>
#include <iomanip>
#include <Windows.h> //Used for GetAsyncKeyState
#include <chrono>  //For timer
#include "PxPhysicsAPI.h"


namespace PhysicsEngine
{
	using namespace std;
	using namespace physx;

	//a list of colours: Circus Palette
	static const PxVec3 color_palette[] = {PxVec3(46.f/255.f,9.f/255.f,39.f/255.f),PxVec3(217.f/255.f,0.f/255.f,0.f/255.f),
		PxVec3(255.f/255.f,45.f/255.f,0.f/255.f),PxVec3(255.f/255.f,140.f/255.f,54.f/255.f),PxVec3(4.f/255.f,117.f/255.f,111.f/255.f)};

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
			BRICK		= (1 << 0),
			FLOOR		= (1 << 1),
			BLADE		= (1 << 2)

		};
	};

	//All game related variables / methods will be stored here.
	struct GameVariables {
		int playerScore = 0; //Tracks the players score, when score target is reached the player has won.
		float timer = 30.f; //Tracks the time remaining for the game, will be outputted in console.     
		int target = 300; //Point target for the player to reach

		//Called when a brick has been pushed out of its spawn position
		void BrickFallen(){
			++playerScore; //Increment score 

			//Console debugging
			std::cout << playerScore << endl;
			std::cout << "brickFell" << endl;
		}

	

	};

	///An example class showing the use of springs (distance joints).
	class Trampoline
	{
		vector<DistanceJoint*> springs;
		Box *bottom, *top;

	public:
		Trampoline(const PxVec3& dimensions=PxVec3(1.f,1.f,1.f), PxReal stiffness=1.f, PxReal damping=1.f)
		{
			PxReal thickness = .1f;
			bottom = new Box(PxTransform(PxVec3(0.f,thickness,0.f)),PxVec3(dimensions.x,thickness,dimensions.z));
			top = new Box(PxTransform(PxVec3(0.f,dimensions.y+thickness,0.f)),PxVec3(dimensions.x,thickness,dimensions.z));
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

		GameVariables* gv;

		MySimulationEventCallback(GameVariables* _gv) : trigger(false),gv(_gv) {}

		///Method called when the contact with the trigger object is detected.
		virtual void onTrigger(PxTriggerPair* pairs, PxU32 count) override
		{
			for (PxU32 i = 0; i < count; ++i) {
				// only count ENTER events
				if ((pairs[i].status & PxPairFlag::eNOTIFY_TOUCH_FOUND)
					&& pairs[i].otherShape->getActor()->getName()
					&& std::string(pairs[i].otherShape->getActor()->getName()) == "Brick")
				{
					gv->BrickFallen();


				}
			}
		}

		///Method called when the contact by the filter shader is detected.
		virtual void onContact(const PxContactPairHeader& hdr, const PxContactPair* pairs, PxU32 count) override
		{

		}

		virtual void onConstraintBreak(PxConstraintInfo *constraints, PxU32 count) {}
		virtual void onWake(PxActor **actors, PxU32 count) {}
		virtual void onSleep(PxActor **actors, PxU32 count) {}
#if PX_PHYSICS_VERSION >= 0x304000
		virtual void onAdvance(const PxRigidBody *const *bodyBuffer, const PxTransform *poseBuffer, const PxU32 count) {}
#endif
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

		pairFlags = PxPairFlag::eCONTACT_DEFAULT | PxPairFlag::eNOTIFY_TOUCH_FOUND;
		return PxFilterFlags();
//		pairFlags |= PxPairFlag::eCCD_LINEAR;
		
		
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

		std::chrono::steady_clock::time_point timerStart;
		Plane* plane;
		Box* box, * box2;
		Box* bulldozer = nullptr;
		PxVec3 dozerPos;
		const PxReal dozerSpeed = 4.0f; //Speed of machine
		GameVariables gv;

		float   mFrontPlaneZ;
		float   mBackPlaneZ;
		float   mFallMargin = 0.1f;

		std::vector<Box*> bricks;

		//Custom Material Pointers
		PxMaterial* concreteMaterial = nullptr;
		PxMaterial* metalMaterial = nullptr;
		PxMaterial* brickMaterial = nullptr;
		PxMaterial* rubberMaterial = nullptr;

		//Bulldozer Wheel / Joint Pointers
		DynamicActor* wheelFR = nullptr;
		DynamicActor* wheelFL = nullptr;
		DynamicActor* wheelBR = nullptr;
		DynamicActor* wheelBL = nullptr;

		PxRevoluteJoint* jointFR = nullptr;
		PxRevoluteJoint* jointFL = nullptr;
		PxRevoluteJoint* jointBR = nullptr;
		PxRevoluteJoint* jointBL = nullptr;

		MySimulationEventCallback* my_callback;
		
	public:
		//specify your custom filter shader here
		//PxDefaultSimulationFilterShader by default
		MyScene() : Scene(CustomFilterShader) {};

		Box* GetBulldozer() const {
			return bulldozer;
		}
		int GetScore() const {
			return gv.playerScore;
		}

		int GetTarget() const {
			return gv.target;
		}

		///A custom scene class
		void SetVisualisation()
		{
			px_scene->setVisualizationParameter(PxVisualizationParameter::eSCALE, 1.0f);
			px_scene->setVisualizationParameter(PxVisualizationParameter::eCOLLISION_SHAPES, 1.0f);

		}

		//Custom scene initialisation
		virtual void CustomInit() 
		{
			SetVisualisation();			



			timerStart = std::chrono::steady_clock::now();

			//Setting values for friction and restitution (bounciness)
			PxMaterial* mat = GetMaterial();
			mat->setDynamicFriction(5.f);
			mat->setStaticFriction(5.f);
			mat->setRestitution(0.0f);

			PxMaterial* rubberMaterial = GetPhysics()->createMaterial(0.9f, 0.9f, 0.1f);   // High grip and slightly bouncy
			PxMaterial* metalMaterial = GetPhysics()->createMaterial(0.6f, 0.5f, 0.02f); //low bounciness
			PxMaterial* concreteMaterial = GetPhysics()->createMaterial(0.8f, 0.7f, 0.05f); //High friction, things may bounce off it but mostly dampened
			PxMaterial* brickMaterial = GetPhysics()->createMaterial(0.9f, 0.9f, 0.1f);    // 0.1 restitution allows for slight bounce



			///Initialise and set the customised event callback
			my_callback = new MySimulationEventCallback(&gv);
			px_scene->setSimulationEventCallback(my_callback);

			//Adds a plane to the simulation, this will act as a floor for the level.
			plane = new Plane();
			plane->Color(PxVec3(210.f/255.f,210.f/255.f,210.f/255.f));
			plane->SetupFiltering(FilterGroup::BRICK, FilterGroup::FLOOR);
			plane->Get()->setName("Plane");
			PxShape* planeShape = plane->GetShape();
			planeShape->setMaterials(&concreteMaterial, 1); //Set the material of the plane to concrete

			Add(plane);

			//Params for bricks and walls
			PxVec3 brickSize(.5f, .25f, .25f); //Size of individual bricks
			float gap = 0.000f; //Create a slight gap between bricks to give the natural look and cracks between bricks
			int rows = 12;
			int fbCols = 10;    //Bricks across front/back
			int sideCols = 14;    //Bricks along each side wall
			float density = 3.0f;
			PxVec3 brickCol = PxVec3(178.f / 255.f, 34.f / 255.f, 34.f / 255.f); //Brick colour, trying to achieve the natural red look

			//Calculate span and centre of the wall
			float spanX = fbCols * (brickSize.x * 2 + gap);
			float spanZ = sideCols * (brickSize.z * 2 + gap);

			float baseY = brickSize.y; //Bottom row of the wall.

			//Variables for the door
			const float doorWidthBricks = 3.0f;    
			const float doorHeight = 4.0f;    


			//Varaiables for the compound shape
			PxVec3 center = PxVec3(0, 0, 10.f + (14 * (0.25f * 2)) / 2.f);
			float spawnOffsetZ = -5.0f;

			PxVec3 bodySize(2.0f, 1.0f, 3.0f);
			PxVec3 bladeSize(3.0f, 0.5f, 1.0f);

			float groundClearance = 0.05f;
			float startY = bodySize.y + groundClearance;

			PxVec3 chassisStart = PxVec3(center.x, startY, center.z - (14 * (0.25f * 2)) / 2.f + 10.0f - spawnOffsetZ);
			PxTransform chassisPose(chassisStart);

			//Create the chassis of the bulldozer
			DynamicActor* chassis = new DynamicActor(chassisPose);
			chassis->CreateShape(PxBoxGeometry(bodySize), 5.0f);
			chassis->Color(PxVec3(0.4f, 0.4f, 0.4f));

			PxRigidDynamic* chassisRigid = static_cast<PxRigidDynamic*>(chassis->Get());
			chassisRigid->setCMassLocalPose(PxTransform(PxVec3(0.0f, -0.3f, 0.0f)));
			chassisRigid->setMass(1000.f); // heavier vehicle
			chassisRigid->setMassSpaceInertiaTensor(PxVec3(500, 2000, 2000)); // increase moment of inertia around X axis

			//Bulldozer blade creation
			float bladeBackwardOffset = 0.0f;
			float bladeLowerOffset = 0.25f;
			PxBoxGeometry bladeGeom(bladeSize);
			PxTransform bladeLocalPose(PxVec3(0.0f, -bladeSize.y - bladeLowerOffset, -bodySize.z - bladeGeom.halfExtents.z - bladeBackwardOffset));

			PxShape* bladeShape = chassisRigid->createShape(bladeGeom, *metalMaterial);
			bladeShape->setLocalPose(bladeLocalPose);
			bladeShape->setMaterials(&metalMaterial, 1);
			bladeShape->setSimulationFilterData(PxFilterData(FilterGroup::BLADE, FilterGroup::BRICK, 0, 0));
			bladeShape->userData = new UserData();
			static PxVec3 bladeColor(0.4f, 0.4f, 0.4f);
			((UserData*)bladeShape->userData)->color = &bladeColor;

			//Wheels are created as seperate dynamic actors

			PxReal wheelRadius = 0.5f;
			PxReal wheelDensity = 2.0f;
			PxSphereGeometry wheelGeom(wheelRadius);

			float clearance = 0.1f; // 5 cm extra outwards
			PxVec3 wheelOffsetFR(bodySize.x - wheelRadius + clearance, -bodySize.y, bodySize.z - wheelRadius + clearance);
			PxVec3 wheelOffsetFL(-bodySize.x + wheelRadius - clearance, -bodySize.y, bodySize.z - wheelRadius + clearance);
			PxVec3 wheelOffsetBR(bodySize.x - wheelRadius + clearance, -bodySize.y, -bodySize.z + wheelRadius - clearance);
			PxVec3 wheelOffsetBL(-bodySize.x + wheelRadius - clearance, -bodySize.y, -bodySize.z + wheelRadius - clearance);

			PxQuat wheelRotation = PxQuat(PxHalfPi, PxVec3(1, 0, 0));

			// Front Right Wheel
			wheelFR = new DynamicActor(PxTransform(chassisPose.p + wheelOffsetFR, wheelRotation));
			wheelFR->CreateShape(wheelGeom, wheelDensity);
			wheelFR->Color(PxVec3(0.1f, 0.1f, 0.1f));
			PxShape* wheelShapeFR = wheelFR->GetShape();
			wheelShapeFR->setMaterials(&rubberMaterial, 1);
			Add(wheelFR);

			// Front Left Wheel
			wheelFL = new DynamicActor(PxTransform(chassisPose.p + wheelOffsetFL, wheelRotation));
			wheelFL->CreateShape(wheelGeom, wheelDensity);
			wheelFL->Color(PxVec3(0.1f, 0.1f, 0.1f));
			PxShape* wheelShapeFL = wheelFL->GetShape();
			wheelShapeFL->setMaterials(&rubberMaterial, 1);
			Add(wheelFL);

			// Back Right Wheel
			wheelBR = new DynamicActor(PxTransform(chassisPose.p + wheelOffsetBR, wheelRotation));
			wheelBR->CreateShape(wheelGeom, wheelDensity);
			wheelBR->Color(PxVec3(0.1f, 0.1f, 0.1f));
			PxShape* wheelShapeBR = wheelBR->GetShape();
			wheelShapeBR->setMaterials(&rubberMaterial, 1);
			Add(wheelBR);

			// Back Left Wheel
			wheelBL = new DynamicActor(PxTransform(chassisPose.p + wheelOffsetBL, wheelRotation));
			wheelBL->CreateShape(wheelGeom, wheelDensity);
			wheelBL->Color(PxVec3(0.1f, 0.1f, 0.1f));
			PxShape* wheelShapeBL = wheelBL->GetShape();
			wheelShapeBL->setMaterials(&rubberMaterial, 1);
			Add(wheelBL);

			//Revolute joints for the wheels
			PxQuat jointFrameRotation = PxQuat(PxHalfPi, PxVec3(1, 0, 0));

			PxTransform localChassisFR(wheelOffsetFR, jointFrameRotation);
			PxTransform localWheel(PxVec3(0), jointFrameRotation);

			jointFR = PxRevoluteJointCreate(*GetPhysics(),
				chassisRigid, localChassisFR,
				static_cast<PxRigidDynamic*>(wheelFR->Get()), localWheel);

			jointFR->setDriveVelocity(0);
			jointFR->setDriveForceLimit(1e6f);
			jointFR->setRevoluteJointFlag(PxRevoluteJointFlag::eDRIVE_ENABLED, true);
			std::cout << "Joint FR drive enabled: " << jointFR->getRevoluteJointFlags().isSet(PxRevoluteJointFlag::eDRIVE_ENABLED) << std::endl;

			PxTransform localChassisFL(wheelOffsetFL, jointFrameRotation);
			jointFL = PxRevoluteJointCreate(*GetPhysics(),
				chassisRigid, localChassisFL,
				static_cast<PxRigidDynamic*>(wheelFL->Get()), localWheel);
			
			jointFL->setDriveVelocity(0);
			jointFL->setDriveForceLimit(1e6f);
			jointFL->setRevoluteJointFlag(PxRevoluteJointFlag::eDRIVE_ENABLED, true);
			std::cout << "Joint FL drive enabled: " << jointFR->getRevoluteJointFlags().isSet(PxRevoluteJointFlag::eDRIVE_ENABLED) << std::endl;

			PxTransform localChassisBR(wheelOffsetBR, jointFrameRotation);
			jointBR = PxRevoluteJointCreate(*GetPhysics(),
				chassisRigid, localChassisBR,
				static_cast<PxRigidDynamic*>(wheelBR->Get()), localWheel);

			jointBR->setDriveVelocity(0);
			jointBR->setDriveForceLimit(1e6f);
			jointBR->setRevoluteJointFlag(PxRevoluteJointFlag::eDRIVE_ENABLED, true);
			std::cout << "Joint BR drive enabled: " << jointFR->getRevoluteJointFlags().isSet(PxRevoluteJointFlag::eDRIVE_ENABLED) << std::endl;

			PxTransform localChassisBL(wheelOffsetBL, jointFrameRotation);
			jointBL = PxRevoluteJointCreate(*GetPhysics(),
				chassisRigid, localChassisBL,
				static_cast<PxRigidDynamic*>(wheelBL->Get()), localWheel);

			jointBL->setDriveVelocity(0);
			jointBL->setDriveForceLimit(1e6f);
			jointBL->setRevoluteJointFlag(PxRevoluteJointFlag::eDRIVE_ENABLED, true);
			std::cout << "Joint BL drive enabled: " << jointFR->getRevoluteJointFlags().isSet(PxRevoluteJointFlag::eDRIVE_ENABLED) << std::endl;


			std::cout
				<< "jointFR enabled: " << (jointFR ? jointFR->getRevoluteJointFlags().isSet(PxRevoluteJointFlag::eDRIVE_ENABLED) : false) << ", "
				<< "jointFL enabled: " << (jointFL ? jointFL->getRevoluteJointFlags().isSet(PxRevoluteJointFlag::eDRIVE_ENABLED) : false) << ", "
				<< "jointBR enabled: " << (jointBR ? jointBR->getRevoluteJointFlags().isSet(PxRevoluteJointFlag::eDRIVE_ENABLED) : false) << ", "
				<< "jointBL enabled: " << (jointBL ? jointBL->getRevoluteJointFlags().isSet(PxRevoluteJointFlag::eDRIVE_ENABLED) : false) << std::endl;

			//Make chassis a member of bulldozer compound actor
			bulldozer = (Box*)chassis;
			Add(bulldozer);


		

			bulldozer->SetKinematic(false);

			// Debugging log
			std::cout << "Bulldozer chassis and wheels created with joints" << std::endl;
			


			//Algorithm to create front and back walls, brick by brick
			for (int y = 0; y < rows; ++y) //Unitl y = amount of rows, keep looping
			{
				bool offset = (y & 1) != 0;
				float  yOff = y * (brickSize.y * 2 + gap);
				for (int x = 0; x < fbCols; ++x) //Until x = amount of collumns, keep looping
				{
					float xOff = (x + (offset ? 0.5f : 0)) * (brickSize.x * 2 + gap);

					//Paramaters for creating a door within the wall
					const float doorWidth = 3.0f;   //Width of doorframe
					const float doorHeight = 4.0f;   //Height of door
					const float halfDoorW = doorWidth * 0.5f; //Calculates half the door width
					const float wallMidX = 0.0f;   
					float brickWorldX = -spanX / 2 + xOff;
					float brickWorldY = baseY + yOff;
					mFrontPlaneZ = center.z - spanZ / 2.f - mFallMargin;
					mBackPlaneZ = center.z + spanZ / 2.f + mFallMargin;

					

					//Front wall, if isDoor is false then proceed and create more bricks
					{
						PxVec3 pos = center + PxVec3(-spanX / 2 + xOff, baseY + yOff, -spanZ / 2);
						Box* b = new Box(PxTransform(pos), brickSize, density); //Create new box actor
						b->SetupFiltering(FilterGroup::BRICK, FilterGroup::FLOOR);
						b->Color(brickCol);
						b->Get()->setName("Brick");
						{
							PxShape* s = b->GetShape();
							PxFilterData fd;
							fd.word0 = FilterGroup::BRICK;
							fd.word1 = FilterGroup::BLADE;
							s->setSimulationFilterData(fd);
							s->setMaterials(&brickMaterial, 1); //Custom Brick Material
						}
						Add(b);
						auto rd = static_cast<PxRigidDynamic*>(b->Get());
						rd->setLinearDamping(1.0f);
						rd->setAngularDamping(1.0f);

						rd->setSleepThreshold(0.0f);
						rd->wakeUp();

						bricks.push_back(b);
					}

					//Back Wall
					{
						PxVec3 pos = center + PxVec3(-spanX / 2 + xOff, baseY + yOff, +spanZ / 2);
						Box* b = new Box(PxTransform(pos), brickSize, density); //Create new box actor
						b->SetupFiltering(FilterGroup::BRICK, FilterGroup::FLOOR);
						b->Color(brickCol);
						b->Get()->setName("Brick");
						{
							PxShape* s = b->GetShape();
							PxFilterData fd;
							fd.word0 = FilterGroup::BRICK;
							fd.word1 = FilterGroup::BLADE;
							s->setSimulationFilterData(fd);
							s->setMaterials(&brickMaterial, 1); //Custom Brick Material
						}
						Add(b);
						auto rd = static_cast<PxRigidDynamic*>(b->Get());
						rd->setLinearDamping(1.0f);
						rd->setAngularDamping(1.0f);

						rd->setSleepThreshold(0.0f);
						rd->wakeUp();
			
						bricks.push_back(b);
					}
				}
			}


			//Left and Right walls brick by brick algoritm
			for (int y = 0; y < rows; ++y)
			{
				bool   offset = (y & 1) != 0;
				float  yOff = y * (brickSize.y * 2 + gap);
				for (int z = 0; z < sideCols; ++z)
				{
					if (z == 0 || z == sideCols - 1)
						continue;

					float zOff = (z + (offset ? 0.5f : 0)) * (brickSize.z * 2 + gap);

					//Left wall bricks
					{
						PxVec3 pos = center + PxVec3(-spanX / 2, baseY + yOff, -spanZ / 2 + zOff);
						Box* b = new Box(PxTransform(pos), brickSize, density);
						b->SetupFiltering(FilterGroup::BRICK, FilterGroup::FLOOR);
						b->Color(brickCol);
						b->Get()->setName("Brick");
						{
							PxShape* s = b->GetShape();
							PxFilterData fd;
							fd.word0 = FilterGroup::BRICK;
							fd.word1 = FilterGroup::BLADE;
							s->setSimulationFilterData(fd);
							s->setMaterials(&brickMaterial, 1); //Custom Brick Material
						}
						Add(b);
						auto rd = static_cast<PxRigidDynamic*>(b->Get());
						rd->setLinearDamping(1.0f);
						rd->setAngularDamping(1.0f);

						rd->setSleepThreshold(0.0f);
						rd->wakeUp();
	
						bricks.push_back(b);
					}

					//Right wall bricks
					{
						PxVec3 pos = center + PxVec3(+spanX / 2, baseY + yOff, -spanZ / 2 + zOff);
						Box* b = new Box(PxTransform(pos), brickSize, density);
						b->SetupFiltering(FilterGroup::BRICK, FilterGroup::FLOOR);
						b->Color(brickCol);//Set the brick colour
						b->Get()->setName("Brick");
						{
							PxShape* s = b->GetShape();
							PxFilterData fd;
							fd.word0 = FilterGroup::BRICK;
							fd.word1 = FilterGroup::BLADE;
							s->setSimulationFilterData(fd);
							s->setMaterials(&brickMaterial, 1); //Custom Brick Material

						}
						Add(b);
						auto rd = static_cast<PxRigidDynamic*>(b->Get());
						rd->setLinearDamping(1.0f);
						rd->setAngularDamping(1.0f);

						rd->setSleepThreshold(0.0f);
						rd->wakeUp();
		
						bricks.push_back(b);


					}
				}
			}


			


			
		}



		//Custom udpate function
		virtual void CustomUpdate() override
		{

			//Timer 

			auto now = std::chrono::steady_clock::now();
			float elapsed = std::chrono::duration<float>(now - timerStart).count();
			float remaining = 51.0f - elapsed;

			if (remaining <= 0.0f) {
				Pause(true);
				std::cout << "Time's up – you lose!\n";
			}

			for (auto it = bricks.begin(); it != bricks.end(); )
			{
				auto rd = static_cast<PxRigidDynamic*>((*it)->Get());
				float z = rd->getGlobalPose().p.z;
				if (z < mFrontPlaneZ || z > mBackPlaneZ)
				{
					gv.BrickFallen();
					it = bricks.erase(it);
				}
				else
					++it;
			}



			float maxMotorForce = 50.f;
			float baseSpeed = 50.0f;
			float turnSpeed = 25.0f;

			// Wake up rigid bodies
			auto WakeUpIfValid = [](DynamicActor* actor) {
				if (actor) {
					PxRigidDynamic* rigid = static_cast<PxRigidDynamic*>(actor->Get());
					if (rigid) rigid->wakeUp();
				}
				};

			WakeUpIfValid(bulldozer);
			WakeUpIfValid(wheelFR);
			WakeUpIfValid(wheelFL);
			WakeUpIfValid(wheelBR);
			WakeUpIfValid(wheelBL);

			float leftSpeed = 0.0f;
			float rightSpeed = 0.0f;

			// Forward/backward drive
			if (GetAsyncKeyState(VK_UP) & 0x8000) {
				leftSpeed -= baseSpeed;
				rightSpeed -= baseSpeed;
			}
			else if (GetAsyncKeyState(VK_DOWN) & 0x8000) {
				leftSpeed += baseSpeed;
				rightSpeed += baseSpeed;
			}

			// Turning left/right by skid steering
			if (GetAsyncKeyState(VK_LEFT) & 0x8000) {
				leftSpeed += turnSpeed;
				rightSpeed -= turnSpeed;
			}
			else if (GetAsyncKeyState(VK_RIGHT) & 0x8000) {
				leftSpeed -= turnSpeed;
				rightSpeed += turnSpeed;
			}

			// Safety check joints
			if (!(jointFR && jointFL && jointBR && jointBL)) {
				std::cerr << "One or more wheel joints are not initialized!" << std::endl;
				return;
			}

			// Apply motor velocities and force limits
			jointFL->setDriveVelocity(leftSpeed);
			jointFL->setDriveForceLimit(maxMotorForce);

			jointBL->setDriveVelocity(leftSpeed);
			jointBL->setDriveForceLimit(maxMotorForce);

			jointFR->setDriveVelocity(rightSpeed);
			jointFR->setDriveForceLimit(maxMotorForce);

			jointBR->setDriveVelocity(rightSpeed);
			jointBR->setDriveForceLimit(maxMotorForce);


			Scene::CustomUpdate();
		}

		/// An example use of key release handling
		void ExampleKeyReleaseHandler()
		{
			cerr << "I am realeased!" << endl;
		}

		/// An example use of key presse handling
		void ExampleKeyPressHandler()
		{
			cerr << "I am pressed!" << endl;
		}
	};
};
