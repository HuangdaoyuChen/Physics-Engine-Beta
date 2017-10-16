#ifndef __PWORLD_H_INCLUDED__
#define __PWORLD_H_INCLUDED__

#include <vector>

#include "precision.h"
#include "core.h"
#include "particle.h"
#include "pfgen.h"
#include "pcontacts.h"


class ParticleWorld
{
public:
	typedef std::vector<Particle*> Particles;
	typedef std::vector<ParticleContactGenerator*> ContactGenerators;

protected:
	Particles particles;
	ContactGenerators contactGenerators;
	ParticleForceRegistry registry;
	ParticleContactResolver resolver;
	ParticleContact *contacts;
	int maxContacts;
	bool calculateIterations;

public:
	// constructor
	ParticleWorld(int maxContacts, int iterations = 0);
	~ParticleWorld();

	// accesor
	Particles& getParticles();
	ContactGenerators& getContactGenerators();
	ParticleForceRegistry& getForceRegistry();

	void startFrame();
	int generateContacts();
	void integrate(real duration);
	void runPhysics(real duration);
};


#endif // __PWORLD_H_INCLUDED__