#ifndef __COLLIDE_COARSE_H_INCLUDED__
#define __COLLIDE_COARSE_H_INCLUDED__


#include <vector>

#include "precision.h"
#include "core.h"
#include "body.h"

struct PotentialContact
{
	RigidBody* body[2];
};

template<class BoundingVolumeClass>
class BVHNode
{
public:
	BVHNode* children[2];
	BVHNode* parent;
	BoundingVolumeClass volume;
	RigidBody* body;

public:
	BVHNode(BVHNode* parent, const BoundingVolumeClass &volume, 
		RigidBody* body = NULL);
	~BVHNode();

	bool isLeaf() const;
	bool overlaps(BVHNode<BoundingVolumeClass>* other) const;
	int getPotentialContacts(PotentialContact* contacts, int limit) const;
	int getPotentialContactsWith(BVHNode<BoundingVolumeClass>* other,
		PotentialContact* contacts, int limit) const;
	void insert(RigidBody* newBdy, const BoundingVolumeClass &newVolume);
	void recalculateBoundingVolume(bool recurse = true);
};

struct BoundingSphere
{
public:
	Vector2 center;
	real radius;

public:
	BoundingSphere(const Vector2 &center, real radius);
	BoundingSphere(const BoundingSphere &one, const BoundingSphere &other);
	bool overlaps(const BoundingSphere *other) const;
	real getGrowth(const BoundingSphere &other) const;
};


#endif // __COLLIDE_COARSE_H_INCLUDED__