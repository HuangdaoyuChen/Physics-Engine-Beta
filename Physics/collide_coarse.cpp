#include "collide_coarse.h"

template<class BoundingVolumeClass>
BVHNode<BoundingVolumeClass>::BVHNode
	(BVHNode* parent, const BoundingVolumeClass &volume, RigidBody* body = NULL)
{
	this->parent = parent;
	this->volume = volume;
	this->body = body;
	children[0] = NULL;
	children[1] = NULL;
}

template<class BoundingVolumeClass>
BVHNode<BoundingVolumeClass>::~BVHNode()
{
	if (parent != NULL)
	{
		BVHNode<BoundingVolumeClass> *sibling;
		if (this == parent->children[0])
			sibling = parent->children[1];
		else
			sibling = parent->children[0];

		parent->volume = sibling->volume;
		parent->body = sibling->body;
		parent->children[0] = sibling->children[0];
		parent->children[1] = sibling->children[1];

		sibling->parent = NULL;
		sibling->volume = NULL;
		sibling->children[0] = NULL;
		sibling->children[1] = NULL;
		delete sibling;

		parent->recalculateBoundingVolume();
	}

	if (children[0] != NULL)
	{
		children[0]->parent = NULL;
		delete children[0];
	}
	if (children[1] != NULL)
	{
		children[1]->parent = NULL;
		delete children[1];
	}
}

template<class BoundingVolumeClass>
bool BVHNode<BoundingVolumeClass>::isLeaf() const
{
	return (body != NULL);
}

template<class BoundingVolumeClass>
bool BVHNode<BoundingVolumeClass>::overlaps(BVHNode<BoundingVolumeClass>* other) const
{
	return volume.overlaps(other->volume); // ?
}

template<class BoundingVolumeClass>
int BVHNode<BoundingVolumeClass>::getPotentialContacts
	(PotentialContact* contacts, int limit) const
{
	if (isLeaf() || limit == 0)
		return 0;
	int count = children[0]->getPotentialContactsWith(children[1], contacts, limit);
	return count;
}

template<class BoundingVolumeClass>
int BVHNode<BoundingVolumeClass>::getPotentialContactsWith
	(BVHNode<BoundingVolumeClass>* other, PotentialContact* contacts, int limit) const
{
	if (!overlaps(other) || limit == 0)
		return 0;

	// potential contact found
	if (isLeaf() && other->isLeaf())
	{
		contacts->body[0] = body;
		contacts->body[1] = other->body;
		return 1;
	}

	int count;
	// traverse the bigger node
	if (other->isLeaf() || !isLeaft() && volume->getSize() > other->volume->getSize())
	{ // traverse this node
		count = children[0]->getPotentialContactsWith(other, contacts, limit);
		if (limit > count)
			count += children[1]->getPotentialContactsWith(other, contacts + count, limit - count);
	}
	else
	{ // traverse other node
		count = getPotentialContactsWith(other->children[0], contacts, limit);
		if (limit > count)
			count += getPotentialContactsWith(other->children[1], contacts + count, limit - count);
	}
	return count;
}

template<class BoundingVolumeClass>
void BVHNode<BoundingVolumeClass>::insert
	(RigidBody* newBody, const BoundingVolumeClass &newVolume)
{
	if (isLeaf())
	{
		BVHNode(this, volume, body);
		BVHNode(this, newVolume, newBody);
		this->body = NULL;
		recalculateBoundingVolume();
	}
	else
	{
		if (children[0]->getGrowth(newVolume) < children[1]->getGrowth(newVolume))
			children[0]->insert(newBody, newVolume);
		else
			children[1]->insert(newBody, newVolume);
	}
}

template<class BoundingVolumeClass>
void BVHNode<BoundingVolumeClass>::recalculateBoundingVolume(bool recurse)
{
	if (isLeaf())
		return;

	volume = BoundingVolumeClass(children[0]->volume, children[1]->volume);

	// recuse up to root
	if (parent != NULL)
		parent->recalculateBoundingVolume(true);
}


BoundingSphere::BoundingSphere(const Vector2 &center, real radius)
{
	this->center = center;
	this->radius = radius;
}

BoundingSphere::BoundingSphere(const BoundingSphere &one, const BoundingSphere &other)
{
	Vector2 centerOffset = one.center - other.center;
	real d = centerOffset.magnitude();
	real radiusDiff = real_abs(one.radius - other.radius);

	if (radiusDiff <= d) // enclosure
	{
		if (one.radius > other.radius)
		{
			center = one.center;
			radius = one.radius;
		}
		else
		{
			center = other.center;
			radius = other.radius;
		}
	}
	else
	{
		radius = (d + one.radius + other.radius) / 2;
		center = one.center + centerOffset.unit() * (radius - one.radius);
	}
}

bool BoundingSphere::overlaps(const BoundingSphere *other) const
{
	return ((center - other->center).magnitude() <= radius + other->radius);
}

real BoundingSphere::getGrowth(const BoundingSphere &other) const
{
	BoundingSphere s(*this, other);
	return (s.radius - radius);
}