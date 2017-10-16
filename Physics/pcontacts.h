#ifndef __PCONTACTS_H_INCLUDED__
#define __PCONTACTS_H_INCLUDED__


#include "precision.h"
#include "core.h"
#include "particle.h"

class ParticleContact
{
	friend class ParticleContactResolver;

public:
	Particle* particle[2];
	real restitution;
	Vector2 contactNormal; // direction away from particle[1]
	real penetration;
	Vector2 particleMovement[2];

public:
	//ParticleContact(Particle* particle[2], real restitution, 
	//Vector2 contactNormal, real penetration);

protected:
	void resolve(real duration);
	real calculateSeparatingVelocity() const;

private:
	real totalInverseMass() const;
	void resolveVelocity(real duration);
	void resolveInterpenetration(real duration);
	real microCollisionResolve(real newSepVelocity, real duration) const;
};


class ParticleContactResolver
{
protected:
	int iterations;
	int iterationsUsed;

public:
	ParticleContactResolver(int iterations);
	void setIterations(int iterations);
	void updatePenetration(ParticleContact *contactArray, 
		int numContacts, int maxIndex);
	void resolveContacts(ParticleContact *contactArray, 
		int numContacts, real duration);
};


class ParticleContactGenerator
{
public:
	virtual int addContact(ParticleContact *contact, int limit) const = 0;
};


#endif // __PCONTACTS_H_INCLUDED__