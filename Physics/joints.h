#ifndef __JOINTS_H_INCLUDED__
#define __JOINTS_H_INCLUDED__


#include "precision.h"
#include "core.h"
#include "body.h"

#include "contacts.h"

class Joint : public ContactGenerator
{
public:
	RigidBody* body[2];
	Vector2 position[2];
	real error;

public:
	Joint();
	Joint(RigidBody *a, const Vector2& a_pos,
		RigidBody *b, const Vector2& b_pos, real error);
	int addContact(Contact *contact, int limit) const;
};

class JointAnchored : public ContactGenerator
{
public:
	RigidBody* body;
	Vector2 position[2];
	real error;

public:
	JointAnchored();
	JointAnchored(RigidBody *a, const Vector2& a_pos,
		const Vector2& b_pos, real error);
	int addContact(Contact *contact, int limit) const;
};

class Link : public ContactGenerator
{
public:
	RigidBody* body[2];

protected:
	real currentLength() const;

public:
	virtual int addContact(Contact *contact, int limit) const = 0;
};

class Rod : public Link
{
public:
	Vector2 position[2];
	real length;

public:
	Rod();
	Rod(RigidBody *a, const Vector2& a_pos,
		RigidBody *b, const Vector2& b_pos, real length);
	int addContact(Contact *contact, int limit) const;
};

class Cable : public Link
{
public:
	Vector2 position[2];
	real length;
	real restitution;

public:
	Cable();
	Cable(RigidBody *a, const Vector2& a_pos,
		RigidBody *b, const Vector2& b_pos, real length, real restitution);
	int addContact(Contact *contact, int limit) const;
};


#endif // __JOINTS_H_INCLUDED__