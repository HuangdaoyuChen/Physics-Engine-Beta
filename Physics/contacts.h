#ifndef __CONTACTS_H_INCLUDED__
#define __CONTACTS_H_INCLUDED__


#include "precision.h"
#include "core.h"
#include "body.h"

class Contact
{
	friend class ContactResolver;
	
public:
	RigidBody* body[2];
	real friction;
	real restitution;
	Vector2 contactPoint;
	Vector2 contactNormal; // from perspective of body[0]
	real penetration;

	Vector2 linearChange[2];
	real angularChange[2];
	Vector2 velocityChange[2];
	real rotationChange[2];

public:
	void setBodyData(RigidBody* body1, RigidBody* body2,
		real friction, real restitution);

protected:
	Matrix2 contactToWorld;
	Vector2 relativeContactPosition[2];
	Vector2 contactVelocity; // local space
	real desiredDeltaVelocity; // local space

protected:
	void calculateInternals(real duration);
	void swapBodies();
	void calculateContactBasis();
	void calculateDesiredDeltaVelocity(real duration);
	Vector2 calculateLocalVelocity(int bodyIndex, real duration);

	Vector2 calculateFrictionLessImpulse();
	Vector2 calculateFrictionImpulse();
	void applyVelocityChange();
	void applyPositionChange(); // resolve penetration

	void matchAwakeState();
};

class ContactResolver
{
protected:
	int positionIteration;
	int velocityIteration;
	real positionEpsilon;
	real velocityEpsilon;

public:
	int positionIterationUsed;
	int velocityIterationUsed;

public:
	ContactResolver(int velocityIteration,
		int positionIteration,
		real velocityEpsilon = (real)0.0,
		real positionEpsilon = (real)0.0);
	void setIterations(int velocityIteration, int positionIteration);
	void resolveContacts(Contact *contactArray,
		int numContacts, real duration);

protected:
	void prepareContacts(Contact *contactArray, 
		int numContacts, real duration);
	void adjustPositions(Contact *contactArray,
		int numContacts, real duration);
	void adjustVelocities(Contact *contactArray,
		int numContacts, real duration);
};

class ContactGenerator
{
public:
	virtual int addContact(Contact *contact, int limit) const = 0;
};


#endif // __CONTACTS_H_INCLUDED__