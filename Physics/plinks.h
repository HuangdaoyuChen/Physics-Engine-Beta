#ifndef __PLINKS_H_INCLUDED__
#define __PLINKS_H_INCLUDED__

#include "precision.h"
#include "core.h"
#include "particle.h"
#include "pcontacts.h"

class ParticleLink : public ParticleContactGenerator
{
public:
	Particle* particle[2];

protected:
	real currentLength() const;

public:
	virtual int addContact(ParticleContact *contact, int limit) const = 0;
};


class ParticleCable : public ParticleLink
{
public:
	real maxLength;
	real restitution;

public:
	ParticleCable();
	ParticleCable(Particle* p1, Particle* p2, real maxLength, real restitution);
	virtual int addContact(ParticleContact *contact, int limit) const;
};


class ParticleRod : public ParticleLink
{
public:
	real length;

public:
	ParticleRod();
	ParticleRod(Particle* p1, Particle* p2, real length);
	virtual int addContact(ParticleContact *contact, int limit) const;
};

class ParticleWall : public ParticleLink
{
public:
	Vector2 normal;

public:
	ParticleWall(Particle* p1, const Vector2 &normal);
	virtual int addContact(ParticleContact *contact, int limit) const;
};


#endif // __PLINKS_H_INCLUDED__