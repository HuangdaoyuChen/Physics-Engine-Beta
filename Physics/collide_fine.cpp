#include "collide_fine.h"

CollisionSphere::CollisionSphere()
{}

CollisionSphere::CollisionSphere(RigidBody *body, real radius)
{
	CollisionSphere::body = body;
	CollisionSphere::radius = radius;
}

CollisionPlane::CollisionPlane()
{}

CollisionPlane::CollisionPlane(const Vector2& normal, real offset)
{
	CollisionPlane::normal = normal;
	CollisionPlane::offset = offset;
}

CollisionBox::CollisionBox()
{}

CollisionBox::CollisionBox(RigidBody *body, const Vector2& halfSize)
{
	CollisionBox::body = body;
	CollisionBox::halfSize = halfSize;
}


CollisionData::CollisionData(int maxContact, real restitution, real friction)
	: MAX_CONTACT(maxContact)
{
	contactArray = new Contact[MAX_CONTACT];
	CollisionData::restitution = restitution;
	CollisionData::friction = friction;
	reset();
}

CollisionData::~CollisionData()
{
	delete[] contactArray;
}

void CollisionData::reset()
{
	contacts = contactArray;
	contactsLeft = MAX_CONTACT;
	contactsCount = 0;
}

void CollisionData::addContacts(int n)
{
	contactsLeft -= n;
	contactsCount += n;
	contacts += n;
}

int CollisionDetector::sphereAndSphere(const CollisionSphere &one,
	const CollisionSphere &two, CollisionData *data)
{
	if (data->contactsLeft <= 0)
		return 0;

	Vector2 positionOne = one.body->getPosition();
	Vector2 positionTwo = two.body->getPosition();
	Vector2 v = positionOne - positionTwo;
	real d = v.magnitude();
	real penetration = one.radius + two.radius - d;

	if (d == 0 || penetration <= 0)
		return 0;
	
	Vector2 normal = v.unit();
	Contact* contact = data->contacts;
	contact->contactPoint = positionOne - normal * (one.radius - penetration); // on edge of two
	contact->contactNormal = normal;
	contact->penetration = penetration;
	contact->setBodyData(one.body, two.body, data->friction, data->restitution);

	data->addContacts(1);
	return 1;
}

int CollisionDetector::sphereAndHalfSpace(const CollisionSphere &sphere,
	const CollisionPlane &plane, CollisionData *data)
{
	if (data->contactsLeft <= 0)
		return 0;

	Vector2 positionSphere = sphere.body->getPosition();
	Vector2 normal = plane.normal.unit();
	real penetration = plane.offset + sphere.radius - positionSphere * normal;

	if (penetration <= 0)
		return 0;

	Contact* contact = data->contacts;
	contact->contactPoint = positionSphere - normal * (sphere.radius - penetration);
	contact->contactNormal = normal;
	contact->penetration = penetration;
	contact->setBodyData(sphere.body, NULL, data->friction, data->restitution);

	data->addContacts(1);
	return 1;
}

int CollisionDetector::sphereAndTruePlane(const CollisionSphere &sphere,
	const CollisionPlane &plane, CollisionData *data)
{
	if (data->contactsLeft <= 0)
		return 0;

	Vector2 positionSphere = sphere.body->getPosition();
	Vector2 normal = plane.normal.unit();
	real distance = positionSphere * normal - plane.offset;
	real penetration = sphere.radius - real_abs(distance);

	if (penetration <= 0)
		return 0;

	if (distance < 0)
		normal.invert();
	
	Contact* contact = data->contacts;
	contact->contactPoint = positionSphere - normal * distance;
	contact->contactNormal = normal;
	contact->penetration = penetration;
	contact->setBodyData(sphere.body, NULL, data->friction, data->restitution);

	data->addContacts(1);
	return 1;
}

int CollisionDetector::boxAndHalfSpace(const CollisionBox &box,
	const CollisionPlane &plane, CollisionData *data)
{
	if (data->contactsLeft <= 0)
		return 0;

	if (false) // todo ealy-out
		return 0;

	Vector2 vertices[4] = 
	{
		Vector2(+box.halfSize.x, +box.halfSize.y),
		Vector2(+box.halfSize.x, -box.halfSize.y),
		Vector2(-box.halfSize.x, +box.halfSize.y),
		Vector2(-box.halfSize.x, -box.halfSize.y)
	};
	Vector2 normal = plane.normal.unit();
	int contactUsed = 0;

	for (int i = 0; i < 4; i++)
	{
		Vector2 vertexPos = box.body->getTransformMatrix() * (vertices[i]);
		real penetration = plane.offset - vertexPos * normal;

		if (penetration > -0.0)
		{
			Contact* contact = data->contacts;
			contact->contactPoint = vertexPos + normal * (penetration);
			contact->contactNormal = normal;
			contact->penetration = penetration;
			contact->setBodyData(box.body, NULL, data->friction, data->restitution);
			data->addContacts(1);
			contactUsed++;
		}
	}
	return contactUsed;
}

int CollisionDetector::boxAndSphere(const CollisionBox &box,
	const CollisionSphere &sphere, CollisionData *data)
{
	if (data->contactsLeft <= 0)
		return 0;

	Vector2 center = sphere.body->getPosition();
	Vector2 relCenter = box.body->getPointInLocalSpace(center);

	real sizeX = real_abs(box.halfSize.x);
	real sizeY = real_abs(box.halfSize.y);
	if (real_abs(relCenter.x) - sphere.radius > sizeX
		|| real_abs(relCenter.y) - sphere.radius > sizeY)
		return 0;

	Vector2 closestPoint;
	closestPoint.x = real_fmax(-sizeX, real_fmin(relCenter.x, sizeX));
	closestPoint.y = real_fmax(-sizeY, real_fmin(relCenter.y, sizeY));

	real penetration = sphere.radius - (closestPoint - relCenter).magnitude();
	if (penetration <= 0)
		return 0;

	// does not handle if sphere is inside box
	Vector2 closestPointWorld = box.body->getPointInWorldSpace(closestPoint);
	Contact* contact = data->contacts;
	contact->contactPoint = closestPointWorld;
	contact->contactNormal = (closestPointWorld - center).unit();
	contact->penetration = penetration;
	contact->setBodyData(box.body, sphere.body, data->friction, data->restitution);

	data->addContacts(1);
	return 1;
}

int CollisionDetector::boxAndPoint(const CollisionBox &box,
	const Vector2 &point, CollisionData *data)
{
	if (data->contactsLeft <= 0)
		return 0;

	Vector2 relPoint = box.body->getPointInLocalSpace(point);
	Vector2 normal;
	real penetration;

	real depthX = box.halfSize.x - real_abs(relPoint.x);
	if (depthX <= 0) 
		return 0;
	normal = box.body->getTransformMatrix().getAxis(0) * -1;
	if (relPoint.x < 0)
		normal.invert();
	penetration = depthX;

	real depthY = box.halfSize.y - real_abs(relPoint.y);
	if (depthY <= 0)
		return 0;
	else if (depthY < depthX)
	{
		normal = box.body->getTransformMatrix().getAxis(1) * -1;
		if (relPoint.y < 0)
			normal.invert();
		penetration = depthY;
	}

	Contact* contact = data->contacts;
	contact->contactPoint = point;
	contact->contactNormal = normal;
	contact->penetration = penetration;
	contact->setBodyData(box.body, NULL, data->friction, data->restitution);

	data->addContacts(1);
	return 1;
}

// todo
int CollisionDetector::boxAndBox(const CollisionBox &one,
	const CollisionBox &two, CollisionData *data)
{
	if (data->contactsLeft <= 0)
		return 0;

	int contactUsed = 0;

	Vector2 verticesOne[4] =
	{
		Vector2(+one.halfSize.x, +one.halfSize.y),
		Vector2(+one.halfSize.x, -one.halfSize.y),
		Vector2(-one.halfSize.x, +one.halfSize.y),
		Vector2(-one.halfSize.x, -one.halfSize.y)
	};
	for (int i = 0; i < 4; i++)
	{
		Vector2 vertexPos = one.body->getTransformMatrix() * (verticesOne[i]);
		int contact = boxAndPoint(two, vertexPos, data);
		if (contact == 1)
		{
			data->contactArray[data->contactsCount - 1].body[1] = one.body;
			contactUsed ++;
		}
	}

	Vector2 verticesTwo[4] =
	{
		Vector2(+two.halfSize.x, +two.halfSize.y),
		Vector2(+two.halfSize.x, -two.halfSize.y),
		Vector2(-two.halfSize.x, +two.halfSize.y),
		Vector2(-two.halfSize.x, -two.halfSize.y)
	};
	for (int i = 0; i < 4; i++)
	{
		Vector2 vertexPos = two.body->getTransformMatrix() * (verticesTwo[i]);
		int contact = boxAndPoint(one, vertexPos, data);
		if (contact == 1)
		{
			data->contactArray[data->contactsCount - 1].body[1] = two.body;
			contactUsed++;
		}
	}

	return contactUsed;
}

Vector2 CollisionPrimitive::getAxis(unsigned index) const
{
	return body->getTransformMatrix().getAxis(index);
}

static inline real transformToAxis(
	const CollisionBox &box,
	const Vector2 &axis
	)
{
	return
		box.halfSize.x * real_abs(axis * box.getAxis(0)) +
		box.halfSize.y * real_abs(axis * box.getAxis(1));
}

/*
* This function checks if the two boxes overlap
* along the given axis, returning the ammount of overlap.
* The final parameter toCentre
* is used to pass in the vector between the boxes centre
* points, to avoid having to recalculate it each time.
*/
static inline real penetrationOnAxis(
	const CollisionBox &one,
	const CollisionBox &two,
	const Vector2 &axis,
	const Vector2 &toCentre
	)
{
	// Project the half-size of one onto axis
	real oneProject = transformToAxis(one, axis);
	real twoProject = transformToAxis(two, axis);

	// Project this onto the axis
	real distance = real_abs(toCentre * axis);

	// Return the overlap (i.e. positive indicates
	// overlap, negative indicates separation).
	return oneProject + twoProject - distance;
}

static inline bool tryAxis(
	const CollisionBox &one,
	const CollisionBox &two,
	Vector2 axis,
	const Vector2& toCentre,
	unsigned index,

	// These values may be updated
	real& smallestPenetration,
	unsigned &smallestCase
	)
{
	// Make sure we have a normalized axis, and don't check almost parallel axes
	if (axis.squareMagnitude() < 0.0001) return true;
	axis.normalize();

	real penetration = penetrationOnAxis(one, two, axis, toCentre);

	if (penetration < -0.0) return false;
	if (penetration < smallestPenetration) {
		smallestPenetration = penetration;
		smallestCase = index;
	}
	return true;
}

void fillPointFaceBoxBox(
	const CollisionBox &one,
	const CollisionBox &two,
	const Vector2 &toCentre,
	CollisionData *data,
	unsigned best,
	real pen
	)
{
	// This method is called when we know that a vertex from
	// box two is in contact with box one.

	Contact* contact = data->contacts;

	// We know which axis the collision is on (i.e. best),
	// but we need to work out which of the two faces on
	// this axis.
	Vector2 normal = one.getAxis(best);
	if (one.getAxis(best) * toCentre > 0)
	{
		normal = normal * -1.0f;
	}

	// Work out which vertex of box two we're colliding with.
	// Using toCentre doesn't work!
	Vector2 vertex = two.halfSize;
	if (two.getAxis(0) * normal < 0) vertex.x = -vertex.x;
	if (two.getAxis(1) * normal < 0) vertex.y = -vertex.y;

	// Create the contact data
	contact->contactNormal = normal;
	contact->penetration = pen;
	contact->contactPoint = two.body->getTransformMatrix() * vertex;
	contact->setBodyData(one.body, two.body,
		data->friction, data->restitution);
}

// This preprocessor definition is only used as a convenience
// in the boxAndBox contact generation method.
#define CHECK_OVERLAP(axis, index) \
    if (!tryAxis(one, two, (axis), toCentre, (index), pen, best)) return 0;

unsigned CollisionDetector::boxAndBox2(
	const CollisionBox &one,
	const CollisionBox &two,
	CollisionData *data
	)
{
	//if (!IntersectionTests::boxAndBox(one, two)) return 0;

	// Find the vector between the two centres
	Vector2 toCentre = two.getAxis(2) - one.getAxis(2);

	// We start assuming there is no contact
	real pen = REAL_MAX;
	unsigned best = 0xffffff;

	// Now we check each axes, returning if it gives us
	// a separating axis, and keeping track of the axis with
	// the smallest penetration otherwise.
	CHECK_OVERLAP(one.getAxis(0), 0);
	CHECK_OVERLAP(one.getAxis(1), 1);

	CHECK_OVERLAP(two.getAxis(0), 3);
	CHECK_OVERLAP(two.getAxis(1), 4);

	// Store the best axis-major, in case we run into almost
	// parallel edge collisions later
	unsigned bestSingleAxis = best;


	// Make sure we've got a result.
	assert(best != 0xffffff);

	// We now know there's a collision, and we know which
	// of the axes gave the smallest penetration. We now
	// can deal with it in different ways depending on
	// the case.
	if (best < 3)
	{
		// We've got a vertex of box two on a face of box one.
		fillPointFaceBoxBox(one, two, toCentre, data, best, pen);
		data->addContacts(1);
		return 1;
	}
	else if (best < 6)
	{
		// We've got a vertex of box one on a face of box two.
		// We use the same algorithm as above, but swap around
		// one and two (and therefore also the vector between their
		// centres).
		fillPointFaceBoxBox(two, one, toCentre*-1.0f, data, best - 3, pen);
		data->addContacts(1);
		return 1;
	}

	return 0;
}
#undef CHECK_OVERLAP