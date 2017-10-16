#include "pcontacts.h"
/*
ParticleContact::ParticleContact(Particle* particle[2], real restitution,
	Vector2 contactNormal, real penetration)
{
	ParticleContact::particle[0] = particle[0];
	ParticleContact::particle[1] = particle[1];
	ParticleContact::restitution = restitution;
	ParticleContact::contactNormal = contactNormal;
	ParticleContact::penetration = penetration;
}
*/
void ParticleContact::resolve(real duration)
{
	contactNormal.normalize();
	resolveVelocity(duration);
	resolveInterpenetration(duration);
}

// vs = (v0 - v2) * n
real ParticleContact::calculateSeparatingVelocity() const
{
	Vector2 relativeVelocity = particle[0]->getVelocity();
	if (particle[1] != NULL)
		relativeVelocity.minus(particle[1]->getVelocity());

	return relativeVelocity * contactNormal;
}

real ParticleContact::totalInverseMass() const
{
	real totalInverseMass = particle[0]->getInverseMass();
	if (particle[1] != NULL)
		totalInverseMass += particle[1]->getInverseMass();
	return totalInverseMass;
}

// removes closing velocity due to acceleration during contact
real ParticleContact::microCollisionResolve(real newSepVelocity, real duration) const
{
	real CorNewSepVelocity = newSepVelocity;
	Vector2 accCausedVelocity = particle[0]->getAcceleration();
	if (particle[1] != NULL)
		accCausedVelocity.minus(particle[1]->getAcceleration());
	real accCausedSepVelocity = accCausedVelocity * contactNormal * duration;

	if (accCausedSepVelocity < 0)
	{
		CorNewSepVelocity += restitution * accCausedSepVelocity;
		if (CorNewSepVelocity < 0)
			CorNewSepVelocity = 0;
	}
	return CorNewSepVelocity;
}

// m0 * v0 + m1 * v1 = m0 * v0' + m1 * v1'
// v0' - v1' = -c * (v0 - v1)
// v0' = v0 + 1/m0 * ((-c(v0-v1) - (v0-v1)) / (1/m0 + 1/m1))
// v1' = v1 + 1/m1 * ((-c(v0-v1) - (v0-v1)) / (1/m0 + 1/m1))
void ParticleContact::resolveVelocity(real duration)
{
	real separatingVelocity = calculateSeparatingVelocity();
	if (separatingVelocity >= 0)
		return;
	real newSepVelocity = -separatingVelocity * restitution;
	// resting contact adjustment
	newSepVelocity = microCollisionResolve(newSepVelocity, duration); 
	real deltaVelocity = newSepVelocity - separatingVelocity; // always positive

	real tim = totalInverseMass();
	if (tim <= 0)
		return;

	Vector2 impulse = contactNormal * (deltaVelocity / tim);
	particle[0]->applyImpulse(impulse);

	if (particle[1] != NULL)
		particle[1]->applyImpulse(impulse * -1);
}

// dx0 + dx1 = p
// m0 * dx0 = m1 * dx1
void ParticleContact::resolveInterpenetration(real duration)
{
	if (penetration <= 0)
		return;
	real tim = totalInverseMass();
	if (tim <= 0)
		return;

	Vector2 movePerIMass = contactNormal * (penetration / tim);

	particleMovement[0] = movePerIMass * particle[0]->getInverseMass();
	if (particle[1] != NULL)
		particleMovement[1] = movePerIMass * -particle[1]->getInverseMass();
	else
		particleMovement[1].clear();

	particle[0]->move(particleMovement[0]);
	if (particle[1] != NULL)
		particle[1]->move(particleMovement[1]);
}

ParticleContactResolver::ParticleContactResolver(int iterations)
{
	ParticleContactResolver::iterations = iterations;
	iterationsUsed = 0;
}

void ParticleContactResolver::setIterations(int iterations)
{
	ParticleContactResolver::iterations = iterations;
}

void ParticleContactResolver::updatePenetration(ParticleContact *contactArray, 
	int numContacts, int maxIndex)
{
	Particle* resolvedParticle0 = contactArray[maxIndex].particle[0];
	Particle* resolvedParticle1 = contactArray[maxIndex].particle[1];
	Vector2 movement0 = contactArray[maxIndex].particleMovement[0];
	Vector2 movement1 = contactArray[maxIndex].particleMovement[1];

	for (int i = 0; i < numContacts; i++)
	{
		Vector2 contactNormal = contactArray[i].contactNormal;
		real deltaPenetration = 0;

		if (contactArray[i].particle[0] == resolvedParticle0)
			contactArray[i].penetration -= (movement0 * contactNormal);
		else if (resolvedParticle1 == contactArray[i].particle[0])
			contactArray[i].penetration -= (movement1 * contactNormal);

		if (contactArray[i].particle[1] != NULL)
		{
			if (contactArray[i].particle[1] == resolvedParticle0)
				contactArray[i].penetration += movement0 * contactNormal;
			else if (contactArray[i].particle[1] == resolvedParticle1)
				contactArray[i].penetration += movement1 * contactNormal;
		}
	}
}

void ParticleContactResolver::resolveContacts(ParticleContact *contactArray, 
	int numContacts, real duration)
{
	iterationsUsed = 0;
	while (iterationsUsed < iterations)
	{
		real max = REAL_MAX;
		int maxIndex = -1;

		for (int i = 0; i < numContacts; i++)
		{
			real sepVel = contactArray[i].calculateSeparatingVelocity();
			// solve for the most closing velocity 
			// with negative closing velocity or positive penetration
			if (sepVel < max && (sepVel < 0 || contactArray[i].penetration > 0))
			{
				max = sepVel;
				maxIndex = i;
			}
		}
		if (maxIndex == -1)
			break;

		contactArray[maxIndex].resolve(duration);
		
		// update penetration for all contacts with resolvedParticles
		updatePenetration(contactArray, numContacts, maxIndex);
		
		iterationsUsed++;
	}
	//std::cout << iterationsUsed << " ";
}