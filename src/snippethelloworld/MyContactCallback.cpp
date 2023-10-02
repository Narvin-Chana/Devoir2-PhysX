#include "MyContactCallback.h"

#include <iostream>
#include <PxRigidDynamic.h>

#include "FilterGroup.h"

void MyContactCallback::onContactModify(physx::PxContactModifyPair* const pairs, physx::PxU32 count)
{
	if (pairs[0].shape[0]->getSimulationFilterData().word0 & FilterGroup::BALL_FLY ||
		pairs[0].shape[1]->getSimulationFilterData().word0 & FilterGroup::BALL_FLY)
	{
		pairs[0].contacts.setTargetVelocity(0, physx::PxVec3(0.0f, 250.0f, 0.0f));
		pairs[0].contacts.setNormal(0, physx::PxVec3(0.0f, 1.0f, 0.0f));
	}
}

void MyContactCallback::onContact(const physx::PxContactPairHeader& pairHeader, const physx::PxContactPair* pairs,
                                  physx::PxU32 nbPairs)
{
	if (pairHeader.pairs->shapes[0]->getSimulationFilterData().word0 & FilterGroup::BALL_SLOW && pairs[0].flags.isSet(
			physx::PxContactPairFlag::eACTOR_PAIR_LOST_TOUCH)
	)
	{
		toSlowDownList.push_back(pairHeader.actors[0]->is<physx::PxRigidDynamic>());
	}
	else if (pairHeader.pairs->shapes[1]->getSimulationFilterData().word0 & FilterGroup::BALL_SLOW && pairs[0].flags.
		isSet(physx::PxContactPairFlag::eACTOR_PAIR_LOST_TOUCH)
	)
	{
		toSlowDownList.push_back(pairHeader.actors[1]->is<physx::PxRigidDynamic>());
	}
}

void MyContactCallback::onConstraintBreak(physx::PxConstraintInfo* constraints, physx::PxU32 count)
{
}

void MyContactCallback::onWake(physx::PxActor** actors, physx::PxU32 count)
{
}

void MyContactCallback::onSleep(physx::PxActor** actors, physx::PxU32 count)
{
}

void MyContactCallback::onTrigger(physx::PxTriggerPair* pairs, physx::PxU32 count)
{
}

void MyContactCallback::onAdvance(const physx::PxRigidBody* const* bodyBuffer, const physx::PxTransform* poseBuffer,
                                  const physx::PxU32 count)
{
}

void MyContactCallback::UpdateSlowDowns()
{
	for (const auto& dynamic : toSlowDownList)
	{
		physx::PxVec3 vec = dynamic->getLinearVelocity();
		dynamic->setLinearVelocity(vec / 2);
	}

	toSlowDownList.clear();
}
