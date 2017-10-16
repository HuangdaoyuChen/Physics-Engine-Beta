#include "pworld.h"

ParticleWorld::ParticleWorld(int maxContacts, int iterations)
	: resolver(iterations)
{
	ParticleWorld::maxContacts = maxContacts;
	contacts = new ParticleContact[maxContacts];
	calculateIterations = (iterations == 0);
}

ParticleWorld::~ParticleWorld()
{
	delete[] contacts;
}

ParticleWorld::Particles& ParticleWorld::getParticles()
{
	return particles;
}

ParticleWorld::ContactGenerators& ParticleWorld::getContactGenerators()
{
	return contactGenerators;
}

ParticleForceRegistry& ParticleWorld::getForceRegistry()
{
	return registry;
}

void ParticleWorld::startFrame()
{
	Particles::iterator i = particles.begin();
	for (; i != particles.end(); i++)
		(*i)->clearAcumulator();
}

int ParticleWorld::generateContacts()
{
	int limit = maxContacts;
	ParticleContact *nextContact = contacts;

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

void ParticleWorld::integrate(real duration)
{
	Particles::iterator i = particles.begin();
	for (; i != particles.end(); i++)
		(*i)->integrate(duration);
}

void ParticleWorld::runPhysics(real duration)
{
	registry.updateForces(duration);
	integrate(duration);
	int usedContacts = generateContacts();
	//std::cout << usedContacts;
	if (calculateIterations)
		resolver.setIterations(usedContacts * 2);
	resolver.resolveContacts(contacts, usedContacts, duration);
}