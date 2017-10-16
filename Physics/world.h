#ifndef __WORLD_H_INCLUDED__
#define __WORLD_H_INCLUDED__


#include <vector>

#include "precision.h"
#include "core.h"
#include "body.h"
#include "fgen.h"
#include "contacts.h"

class World
{
public:
	typedef std::vector<RigidBody*> RigidBodies;
	typedef std::vector<ContactGenerator*> ContactGenerators;

protected:
	RigidBodies bodies;
	ContactGenerators contactGenerators;
	ForceRegistry registry;
	ContactResolver resolver;
	Contact *contacts;
	int maxContacts;
	bool calculateIterations;

public:
	// constructor
	World(int maxContcts, int iterations = 0);
	~World();

	// accesor
	RigidBodies& getRigidBodies();
	ContactGenerators& getContactGenerators();
	ForceRegistry& getForceRegistry();

	void startFrame();
	int generateContacts();
	void integrate(real duration);
	void runPhysics(real duration);
};


#endif // __WORLD_H_INCLUDED__