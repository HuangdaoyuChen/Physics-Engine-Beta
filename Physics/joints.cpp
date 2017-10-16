#include "joints.h"

Joint::Joint()
{}

Joint::Joint(RigidBody *a, const Vector2& a_pos,
	RigidBody *b, const Vector2& b_pos, real error)
{
	body[0] = a;
	body[1] = b;
	position[0] = a_pos;
	position[1] = b_pos;
	Joint::error = error;
}

int Joint::addContact(Contact *contact, int limit) const
{
	Vector2 a_pos_world = body[0]->getPointInWorldSpace(position[0]);
	Vector2 b_pos_world = body[1]->getPointInWorldSpace(position[1]);

	Vector2 normal = b_pos_world - a_pos_world;
	real penetration = normal.magnitude() - error;

	if (penetration <= 0)
		return 0;

	contact->setBodyData(body[0], body[1], 100, 0);
	contact->contactPoint = (a_pos_world + b_pos_world) * 0.5f;
	contact->contactNormal = normal.unit();
	contact->penetration = penetration;

	return 1;
}

JointAnchored::JointAnchored()
{}

JointAnchored::JointAnchored(RigidBody *a, const Vector2& a_pos,
	const Vector2& b_pos, real error)
{
	body = a;
	position[0] = a_pos;
	position[1] = b_pos;
	JointAnchored::error = error;
}

int JointAnchored::addContact(Contact *contact, int limit) const
{
	Vector2 a_pos_world = body->getPointInWorldSpace(position[0]);
	Vector2 b_pos_world = position[1];

	Vector2 normal = b_pos_world - a_pos_world;
	real penetration = normal.magnitude() - error;

	if (penetration <= 0)
		return 0;

	contact->setBodyData(body, NULL, 100, 0);
	contact->contactPoint = (a_pos_world + b_pos_world) * 0.5f;
	contact->contactNormal = normal.unit();
	contact->penetration = penetration;

	return 1;
}

real Link::currentLength() const
{
	Vector2 d = body[0]->getPosition() - body[1]->getPosition();
	return d.magnitude();
}

Rod::Rod()
{}

Rod::Rod(RigidBody *a, const Vector2& a_pos,
	RigidBody *b, const Vector2& b_pos, real length)
{
	body[0] = a;
	body[1] = b;
	position[0] = a_pos;
	position[1] = b_pos;
	Rod::length = length;
}

int Rod::addContact(Contact *contact, int limit) const
{
	Vector2 a_pos_world = body[0]->getPointInWorldSpace(position[0]);
	Vector2 b_pos_world = body[1]->getPointInWorldSpace(position[1]);
	Vector2 d = b_pos_world - a_pos_world;
	real currentLen = d.magnitude();
	Vector2 n = d.unit();

	if (currentLen == length)
		return 0;

	contact->setBodyData(body[0], body[1], 0, 0);
	contact->contactPoint = (a_pos_world + b_pos_world) * 0.5f;

	if (currentLen > length)
	{
		contact->contactNormal = n;
		contact->penetration = currentLen - length;
	}
	else
	{
		contact->contactNormal = n * -1;
		contact->penetration = length - currentLen;
	}

	return 1;
}

Cable::Cable()
{}

Cable::Cable(RigidBody *a, const Vector2& a_pos,
	RigidBody *b, const Vector2& b_pos, real length, real restitution)
{
	body[0] = a;
	body[1] = b;
	position[0] = a_pos;
	position[1] = b_pos;
	Cable::length = length;
	Cable::restitution = restitution;
}

int Cable::addContact(Contact *contact, int limit) const
{
	Vector2 a_pos_world = body[0]->getPointInWorldSpace(position[0]);
	Vector2 b_pos_world = body[1]->getPointInWorldSpace(position[1]);
	Vector2 d = b_pos_world - a_pos_world;
	real currentLen = d.magnitude();
	Vector2 n = d.unit();

	if (currentLen <= length)
		return 0;

	contact->setBodyData(body[0], body[1], 0, restitution);
	contact->contactPoint = (a_pos_world + b_pos_world) * 0.5f;
	contact->contactNormal = n;
	contact->penetration = currentLen - length;

	return 1;
}