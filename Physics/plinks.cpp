#include "plinks.h"

real ParticleLink::currentLength() const
{
	Vector2 d = particle[0]->getPosition() - particle[1]->getPosition();
	return d.magnitude();
}

ParticleCable::ParticleCable()
{}

ParticleCable::ParticleCable(Particle* p1, Particle* p2, real maxLength, real restitution)
{
	particle[0] = p1;
	particle[1] = p2;
	ParticleCable::maxLength = maxLength;
	ParticleCable::restitution = restitution;
}

int ParticleCable::addContact(ParticleContact *contact, int limit) const
{
	real currentLen = currentLength();
	if (currentLen < maxLength)
		return 0;

	contact->particle[0] = particle[0];
	contact->particle[1] = particle[1];
	contact->restitution = restitution;
	// particle[0] bounce towards particle[1] (opposite contact normal)
	Vector2 n = particle[1]->getPosition() - particle[0]->getPosition();
	n.normalize();
	contact->contactNormal = n;
	contact->penetration = currentLen - maxLength;
	return 1;
}

ParticleRod::ParticleRod()
{}

ParticleRod::ParticleRod(Particle* p1, Particle* p2, real length)
{
	particle[0] = p1;
	particle[1] = p2;
	ParticleRod::length = length;
}

int ParticleRod::addContact(ParticleContact *contact, int limit) const
{
	real currentLen = currentLength();
	if (currentLen == length)
		return 0;

	contact->particle[0] = particle[0];
	contact->particle[1] = particle[1];
	contact->restitution = 0;
	Vector2 n = particle[1]->getPosition() - particle[0]->getPosition();
	n.normalize();

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

ParticleWall::ParticleWall(Particle* p1, const Vector2 &normal)
{
	particle[0] = p1;
	particle[1] = NULL;
	this->normal = normal;
}

int ParticleWall::addContact(ParticleContact *contact, int limit) const
{
	return 0;
}