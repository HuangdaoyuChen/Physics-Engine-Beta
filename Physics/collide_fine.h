#ifndef __COLLIDE_FINE_H_INCLUDED__
#define __COLLIDE_FINE_H_INCLUDED__


#include <assert.h>

#include "precision.h"
#include "core.h"
#include "body.h"
#include "contacts.h"

class CollisionPrimitive
{
public:
	RigidBody *body;
	//Matrix3 offset;

public:
	Vector2 CollisionPrimitive::getAxis(unsigned index) const;
};

class CollisionSphere : public CollisionPrimitive
{
public:
	real radius;

public:
	CollisionSphere();
	CollisionSphere(RigidBody *body, real radius);
};

class CollisionPlane
{
public:
	Vector2 normal;
	real offset;

public:
	CollisionPlane();
	CollisionPlane(const Vector2& normal, real offset);
};

class CollisionBox : public CollisionPrimitive
{
public:
	Vector2 halfSize;

public:
	CollisionBox();
	CollisionBox(RigidBody *body, const Vector2& halfSize);
};


struct CollisionData
{
	const int MAX_CONTACT;
	Contact *contactArray;
	Contact *contacts;
	int contactsLeft;
	int contactsCount;
	real restitution;
	real friction;

	CollisionData(int maxContact, real restitution, real friction);
	~CollisionData();
	void reset();
	void addContacts(int n);
};

class CollisionDetector
{
public:
	static int sphereAndSphere(const CollisionSphere &one,
		const CollisionSphere &two, CollisionData *data);
	static int sphereAndHalfSpace(const CollisionSphere &sphere,
		const CollisionPlane &plane, CollisionData *data);
	static int sphereAndTruePlane(const CollisionSphere &sphere,
		const CollisionPlane &plane, CollisionData *data);
	static int boxAndHalfSpace(const CollisionBox &box,
		const CollisionPlane &plane, CollisionData *data);
	static int boxAndSphere(const CollisionBox &box, 
		const CollisionSphere &sphere, CollisionData *data);
	static int boxAndPoint(const CollisionBox &box,
		const Vector2 &point, CollisionData *data);
	static int boxAndBox(const CollisionBox &one,
		const CollisionBox &two, CollisionData *data);

	static unsigned boxAndBox2(const CollisionBox &one,
		const CollisionBox &two, CollisionData *data);
};


#endif // __COLLIDE_FINE_H_INCLUDED__