#include "aActor.h"

#pragma warning(disable : 4018)



/****************************************************************
*
*    	    Actor functions
*
****************************************************************/

AActor::AActor() 
{
	m_pInternalSkeleton = new ASkeleton();
	m_pSkeleton = m_pInternalSkeleton;

	m_BVHController = new BVHController();
	m_BVHController->setActor(this);

	m_IKController = new IKController();
	m_IKController->setActor(this);

	// code to update additional Actor data goes here
	resetGuide();

}

AActor::AActor(const AActor* actor)
{
	*this = *actor;
}

AActor& AActor::operator = (const AActor& actor)
{
	// Performs a deep copy
	if (&actor == this)
	{
		return *this;
	}
	m_pSkeleton = actor.m_pSkeleton;

	// code to update additional Actor data goes here


	return *this;
}

AActor::~AActor()
{
	 delete m_IKController;
	 delete m_BVHController;
	 delete m_pInternalSkeleton;

}

void AActor::clear()
{
	// looks like it is clearing more times than the number of actors.  as a result, m_pSkeleton is not defined for last case.
	m_pSkeleton->clear();  

	// code to update additional Actor data goes here
}

void AActor::update()
{
	if (!m_pSkeleton->getRootNode() )
		 return; // Nothing loaded
	else m_pSkeleton->update();

	// code to update additional Actor data goes here

}

ASkeleton* AActor::getSkeleton()
{
	return m_pSkeleton;
}

void AActor::setSkeleton(ASkeleton* pExternalSkeleton)
{
	m_pSkeleton = pExternalSkeleton;
}

void AActor::resetSkeleton()
{
	m_pSkeleton = m_pInternalSkeleton;
}

BVHController* AActor::getBVHController()
{
	return m_BVHController;
}

IKController* AActor::getIKController()
{
	return m_IKController;
}

void AActor::updateGuideJoint(vec3 guideTargetPos)
{
	if (!m_pSkeleton->getRootNode()) { return; }
	// 1.	Set the global position of the guide joint to the global position of the root joint
	AJoint* root = m_pSkeleton->getRootNode();
	vec3 rootPos = root->getGlobalTranslation();
	vec3 guidePos = m_Guide.getGlobalRotation() * rootPos + m_Guide.getGlobalTranslation();

	// 2.	Set the y component of the guide position to 0
	guidePos[1] = 0.0;
	m_Guide.setGlobalTranslation(guidePos);
	
	// 3.	Set the global rotation of the guide joint towards the guideTarget
	vec3 look = m_Guide.getGlobalRotation()[2];
	vec3 toTarget = (vec3(guideTargetPos[0], 0.0, guideTargetPos[2]) - guidePos).Normalize();
	toTarget[0] = -toTarget[0];
	float cosAngle = Dot(look, toTarget);
	float angle = cosAngle > 1.0 - DBL_EPSILON ? 0.0 : cosAngle < -1.0 + DBL_EPSILON ? M_PI : acos(cosAngle);
	mat3 rotY = mat3(1.0);
	rotY[0] = vec3(cos(angle), 0.0, sin(angle));
	rotY[1] = vec3(0.0, 1.0, 0.0);
	rotY[2] = vec3(-sin(angle), 0.0, cos(angle));
	m_Guide.setGlobalRotation(m_Guide.getGlobalRotation() * rotY);
}

void AActor::solveFootIK(float leftHeight, float rightHeight, bool rotateLeft, bool rotateRight, vec3 leftNormal, vec3 rightNormal)
{
	if (!m_pSkeleton->getRootNode()) { return; }
	AJoint* leftFoot = m_pSkeleton->getJointByID(m_IKController->mLfootID);
	AJoint* rightFoot = m_pSkeleton->getJointByID(m_IKController->mRfootID);

	// The normal and the height given are in the world space

	// 1.	Update the local translation of the root based on the left height and the right height
	AJoint* root = m_pSkeleton->getRootNode();
	vec3 rootPos = root->getLocalTranslation();
	rootPos[1] += leftHeight > rightHeight ? leftHeight : rightHeight;
	root->setLocalTranslation(rootPos);
	m_pSkeleton->update();

	// 2.	Update the character with Limb-based IK 
	ATarget targetLfoot = ATarget();
	targetLfoot.setGlobalTranslation(leftFoot->getGlobalTranslation());
	AIKchain mLfootIKchain = m_IKController->createIKchain(m_IKController->mLfootID, 3, m_pSkeleton);
	m_IKController->computeLimbIK(targetLfoot, mLfootIKchain, axisX, m_pSkeleton);

	ATarget targetRfoot = ATarget();
	targetRfoot.setGlobalTranslation(rightFoot->getGlobalTranslation());
	AIKchain mRfootIKchain = m_IKController->createIKchain(m_IKController->mRfootID, 3, m_pSkeleton);
	m_IKController->computeLimbIK(targetRfoot, mRfootIKchain, axisX, m_pSkeleton);

	// Rotate Foot
	vec3 look = vec3(m_Guide.getGlobalRotation()[2]).Normalize();
	if (rotateLeft)
	{
		// Update the local orientation of the left foot based on the left normal
		mat3 lfootRot = leftFoot->getLocalRotation();
		leftNormal = leftNormal.Normalize();
		float angle = acos(Dot(leftNormal, vec3(0.0, 1.0, 0.0)));
		vec3 axis = leftNormal.Cross(vec3(0.0, 1.0, 0.0));
		axis = (leftFoot->getLocal2Global().Inverse() * axis).Normalize();
		leftFoot->setLocalRotation(lfootRot.Rotation3D(axis, angle));
	}
	if (rotateRight)
	{
		// Update the local orientation of the right foot based on the right normal
		mat3 rfootRot = rightFoot->getLocalRotation();
		rightNormal = rightNormal.Normalize();
		float angle = acos(Dot(rightNormal, vec3(0.0, 1.0, 0.0)));
		vec3 axis = rightNormal.Cross(vec3(0.0, 1.0, 0.0));
		axis = (rightFoot->getLocal2Global().Inverse() * axis).Normalize();
		rightFoot->setLocalRotation(rfootRot.Rotation3D(axis, angle));
	}
	m_pSkeleton->update();
}
