#include <ctype.h>
#include <iostream>
#include <vector>

#include "FilterGroup.h"
#include "MyContactCallback.h"
#include "PxPhysicsAPI.h"
#include "../snippetcommon/SnippetPrint.h"
#include "../snippetcommon/SnippetPVD.h"
#include "../snippetrender/SnippetCamera.h"

using namespace physx;

static PxDefaultAllocator gAllocator;
static PxDefaultErrorCallback gErrorCallback;
static PxFoundation* gFoundation = NULL;
static PxPhysics* gPhysics = NULL;
static PxDefaultCpuDispatcher* gDispatcher = NULL;
static PxScene* gScene = NULL;
static PxMaterial* gMaterial = NULL;
static PxPvd* gPvd = NULL;

static int ballCount = 0;
static std::vector<PxRigidDynamic*> bodyList;
static MyContactCallback* callback;

PxFilterFlags FilterShader(
	PxFilterObjectAttributes attributes0, PxFilterData filterData0,
	PxFilterObjectAttributes attributes1, PxFilterData filterData1,
	PxPairFlags& pairFlags, const void* constantBlock, PxU32 constantBlockSize)
{
	if ((filterData0.word0 == FilterGroup::BALL_SLOW) ||
		(filterData1.word0 == FilterGroup::BALL_SLOW))
	{
		pairFlags |= PxPairFlag::eNOTIFY_TOUCH_FOUND | PxPairFlag::eTRIGGER_DEFAULT;
		return PxFilterFlag::eDEFAULT;
	}

	// let triggers through
	if (PxFilterObjectIsTrigger(attributes0) || PxFilterObjectIsTrigger(attributes1) ||
		(filterData0.word0 == FilterGroup::BALL_THROUGH) ||
		(filterData1.word0 == FilterGroup::BALL_THROUGH))
	{
		pairFlags = PxPairFlag::eTRIGGER_DEFAULT;
		return PxFilterFlag::eDEFAULT;
	}

	// generate contacts for all that were not filtered above
	pairFlags = PxPairFlag::eCONTACT_DEFAULT;

	// trigger the contact callback for pairs (A,B) where 
	// the filtermask of A contains the ID of B and vice versa.
	if (filterData0.word0 & filterData1.word1 && filterData1.word0 & filterData0.word1)
		pairFlags |= PxPairFlag::eMODIFY_CONTACTS | PxPairFlag::eNOTIFY_CONTACT_POINTS;

	return PxFilterFlag::eDEFAULT;
}

static PxRigidDynamic* createBallBounce(const PxTransform& t, const PxGeometry& geometry,
                                        const PxVec3& velocity = PxVec3(0))
{
	PxFilterData filterData;
	filterData.word0 = {FilterGroup::BALL_BOUNCE};
	filterData.word1 = {FilterGroup::WALL};

	PxShape* shape = gPhysics->createShape(geometry, *gMaterial);
	PxRigidDynamic* dynamic = gPhysics->createRigidDynamic(t);
	shape->setSimulationFilterData(filterData);

	dynamic->setAngularDamping(0.5f);
	dynamic->setLinearVelocity(velocity);
	dynamic->attachShape(*shape);

	gScene->addActor(*dynamic);

	shape->release();

	return dynamic;
}

static PxRigidDynamic* createBallIgnore(const PxTransform& t, const PxGeometry& geometry,
                                        const PxVec3& velocity = PxVec3(0))
{
	PxFilterData filterData;
	filterData.word0 = {FilterGroup::BALL_THROUGH};
	filterData.word1 = {FilterGroup::WALL};

	PxShape* shape = gPhysics->createShape(geometry, *gMaterial);
	PxRigidDynamic* dynamic = gPhysics->createRigidDynamic(t);
	shape->setSimulationFilterData(filterData);

	dynamic->setAngularDamping(0.5f);
	dynamic->setLinearVelocity(velocity);
	dynamic->attachShape(*shape);

	gScene->addActor(*dynamic);

	shape->release();

	return dynamic;
}


static void createWall(const PxTransform& t, PxU32 size, PxReal halfExtent)
{
	PxFilterData filterData;
	filterData.word0 = {FilterGroup::WALL};
	filterData.word1 = {
		FilterGroup::BALL_BOUNCE | FilterGroup::BALL_FLY | FilterGroup::BALL_THROUGH | FilterGroup::BALL_SLOW
	};

	PxShape* shape = gPhysics->createShape(PxBoxGeometry(halfExtent, halfExtent, 1), *gMaterial);

	shape->setSimulationFilterData(filterData);

	const PxTransform localTm(PxVec3(PxReal(10) - PxReal(size), PxReal(1), 0) * halfExtent);
	PxRigidStatic* body = gPhysics->createRigidStatic(t.transform(localTm));
	body->attachShape(*shape);

	gScene->addActor(*body);

	shape->release();
}

static void createBallFlyUp(const PxTransform& camera)
{
	PxFilterData filterData;
	filterData.word0 = {FilterGroup::BALL_FLY};
	filterData.word1 = {FilterGroup::WALL};

	PxShape* shape = gPhysics->createShape(PxSphereGeometry(3.0f), *gMaterial);

	shape->setSimulationFilterData(filterData);

	const PxVec3 transform(PxVec3(-10, -4, -10) * 20);
	PxRigidDynamic* body = gPhysics->createRigidDynamic(camera);

	body->setLinearVelocity(transform);

	body->attachShape(*shape);

	gScene->addActor(*body);

	shape->release();
}

static void createBallSlow(const PxTransform& camera)
{
	PxFilterData filterData;
	filterData.word0 = {FilterGroup::BALL_SLOW};
	filterData.word1 = {FilterGroup::WALL};

	PxShape* shape = gPhysics->createShape(PxSphereGeometry(3.0f), *gMaterial);

	shape->setSimulationFilterData(filterData);

	const PxVec3 transform(PxVec3(-10, -4, -10) * 20);
	PxRigidDynamic* body = gPhysics->createRigidDynamic(camera);

	body->setLinearVelocity(transform);

	body->attachShape(*shape);

	gScene->addActor(*body);

	shape->release();

	bodyList.push_back(body);
}

void initPhysics(bool interactive)
{
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);

	gPvd = PxCreatePvd(*gFoundation);
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
	gPvd->connect(*transport, PxPvdInstrumentationFlag::eALL);

	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true, gPvd);

	callback = new MyContactCallback();

	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	sceneDesc.gravity = PxVec3(0.0f, 0.0f, 0.0f);
	gDispatcher = PxDefaultCpuDispatcherCreate(2);
	sceneDesc.cpuDispatcher = gDispatcher;
	sceneDesc.filterShader = FilterShader;
	sceneDesc.contactModifyCallback = callback;
	sceneDesc.simulationEventCallback = callback;

	gScene = gPhysics->createScene(sceneDesc);

	PxPvdSceneClient* pvdClient = gScene->getScenePvdClient();
	if (pvdClient)
	{
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
	}
	gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.6f);

	PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0, 1, 0, 0), *gMaterial);
	gScene->addActor(*groundPlane);


	createWall(PxTransform(PxVec3(2, 0, 5.0f)), 10, 25.0f);
}

void stepPhysics(bool /*interactive*/)
{
	gScene->simulate(1.0f / 60.0f);
	gScene->fetchResults(true);

	callback->UpdateSlowDowns();
}

void cleanupPhysics(bool /*interactive*/)
{
	PX_RELEASE(gScene);
	PX_RELEASE(gDispatcher);
	PX_RELEASE(gPhysics);
	if (gPvd)
	{
		PxPvdTransport* transport = gPvd->getTransport();
		gPvd->release();
		gPvd = NULL;
		PX_RELEASE(transport);
	}
	PX_RELEASE(gFoundation);

	printf("SnippetHelloWorld done.\n");
}

void keyPress(unsigned char key, const PxTransform& camera)
{
	switch (toupper(key))
	{
	case ' ':
		switch (ballCount)
		{
		case 0:
			createBallIgnore(camera, PxSphereGeometry(3.0f), PxVec3(-10, -4, -10) * 20);
			break;
		case 1:
			createBallBounce(camera, PxSphereGeometry(3.0f), PxVec3(-10, -4, -10) * 20);
			break;
		case 2:
			createBallFlyUp(camera);
			break;
		case 3:
			createBallSlow(camera);
			break;
		}

		createBallSlow(camera);
		ballCount = (ballCount + 1) % 4;
		break;
	}
}

int snippetMain(int, const char* const*)
{
#ifdef RENDER_SNIPPET
	extern void renderLoop();
	renderLoop();
#else
	static const PxU32 frameCount = 100;
	initPhysics(false);
	for(PxU32 i=0; i<frameCount; i++)
		stepPhysics(false);
	cleanupPhysics(false);
#endif

	return 0;
}
