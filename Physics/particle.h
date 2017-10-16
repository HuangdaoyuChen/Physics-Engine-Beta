#ifndef __PARTICLE_H_INCLUDED__
#define __PARTICLE_H_INCLUDED__


#include "precision.h"
#include "core.h"

class Particle
{
private:
	Vector2 position;
	Vector2 velocity;
	Vector2 acceleration;
	real inverseMass; // 1/m
	real damping; // 0 to 1
	Vector2 forceAccum;

public:
	// constructors
	Particle();
	Particle(const Vector2& position, real inverseMass);

	// accessors
	Vector2 getPosition() const;
	Vector2 getVelocity() const;
	Vector2 getAcceleration() const;
	real getInverseMass() const;
	real getMass() const;

	void integrate(real duration);
	void clearAcumulator();
	void addForce(const Vector2& force);
	bool isFiniteMass() const;
	void applyImpulse(const Vector2& impulse);
	void move(const Vector2& displacement); // teleport
};


#endif // __PARTICLE_H_INCLUDED__