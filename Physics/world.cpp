#include "world.h"

World::World(int maxContacts, int iterations)
	: resolver(iterations, iterations, 0, 0)
{
	World::maxContacts = maxContacts;
	contacts = new Contact[maxContacts];
	calculateIterations = (iterations == 0);
}

World::~World()
{
	delete[] contacts;
}

World::RigidBodies& World::getRigidBodies()
{
	return bodies;
}

World::ContactGenerators& World::getContactGenerators()
{
	return contactGenerators;
}

ForceRegistry& World::getForceRegistry()
{
	return registry;
}

void World::startFrame()
{
	RigidBodies::iterator i = bodies.begin();
	for (; i != bodies.end(); i++)
	{
		(*i)->clearAccumulators();
		(*i)->calculateDerivedData();
	}
}

int World::generateContacts()
{
	int limit = maxContacts;
	Contact *nextContact = contacts;

	ContactGenerators::iterator i = contactGenerators.begin();
	for (; i != contactGenerators.end(); i++)
	{
		if (limit <= 0)
			break;

		int used = (*i)->addContact(nextContact, limit);
		limit -= used;
		nextContact += used;
	}

	return maxContacts - limit;
}

void World::integrate(real duration)
{
	RigidBodies::iterator i = bodies.begin();
	for (; i != bodies.end(); i++)
		(*i)->integrate(duration);
}

void World::runPhysics(real duration)
{
	registry.updateForces(duration);
	integrate(duration);
	int usedContacts = generateContacts();
	//std::cout << usedContacts;
	if (calculateIterations)
		resolver.setIterations(usedContacts * 2, usedContacts * 2);
	resolver.resolveContacts(contacts, usedContacts, duration);
}