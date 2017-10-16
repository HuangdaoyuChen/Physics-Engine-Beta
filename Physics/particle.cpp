#include "particle.h"

Particle::Particle()
{
	damping = (real)0.99;
	inverseMass = 1;
}

Particle::Particle(const Vector2& position, real inverseMass)
{
	Particle::position = position;
	Particle::inverseMass = inverseMass;
	damping = (real)0.99;
}


Vector2 Particle::getPosition() const
{
	return position;
}

Vector2 Particle::getVelocity() const
{
	return velocity;
}

Vector2 Particle::getAcceleration() const
{
	return acceleration;
}

real Particle::getInverseMass() const
{
	return inverseMass;
}

real Particle::getMass() const
{
	return 1 / inverseMass;
}


void Particle::integrate(real duration)
{
	// F = m*a
	//Vector2 resultingAcc = acceleration;
	//resultingAcc.addScaledVector(forceAccum, inverseMass);
	acceleration = forceAccum * inverseMass;
	// v = v + a*t
	velocity.addScaledVector(acceleration, duration);
	// v = v * d^t
	velocity.scale(real_pow(damping, duration));
	// p = p + v*t     + (1/2 * a*t^2) ~ 0
	position.addScaledVector(velocity, duration);
	position.add(acceleration * (duration * duration / 2));
	// clear forces
	clearAcumulator();
}

void Particle::clearAcumulator()
{
	forceAccum.clear();
}

void Particle::addForce(const Vector2& force)
{
	forceAccum.add(force);
}

bool Particle::isFiniteMass() const
{
	return (inverseMass != 0);
}

// v = v0 + p / m
void Particle::applyImpulse(const Vector2& impulse)
{
	velocity.add(impulse *  inverseMass);
}

void Particle::move(const Vector2& displacement)
{
	position.add(displacement);
}