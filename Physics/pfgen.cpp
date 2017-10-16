#include "pfgen.h"

void ParticleForceRegistry::add(Particle *particle, ParticleForceGenerator *fg)
{
	ParticleForceRegistration registration;
	registration.particle = particle;
	registration.fg = fg;
	registrations.push_back(registration);
}

void ParticleForceRegistry::updateForces(real duration)
{
	Registry::iterator i = registrations.begin();
	for (; i != registrations.end(); i++)
		i->fg->updateForce(i->particle, duration);
}

ParticleGravity::ParticleGravity()
{}

ParticleGravity::ParticleGravity(const Vector2& g)
{
	gravity = g;
}

void ParticleGravity::setGravity(const Vector2& gravity)
{
	this->gravity = gravity;
}

void ParticleGravity::updateForce(Particle *particle, real duration)
{
	if (!particle->isFiniteMass())
		return;

	// Fg = m*g
	particle->addForce(gravity * particle->getMass());
}

ParticleDrag::ParticleDrag()
{}

ParticleDrag::ParticleDrag(real k1, real k2)
{
	this->k1 = k1;
	this->k2 = k2;
}

void ParticleDrag::updateForce(Particle *particle, real duration)
{
	Vector2 force;
	Vector2 v = particle->getVelocity();

	// Fdrag = -norm(v)(k1*abs(v) + k2*abs(v)^2)
	real speed = v.magnitude();
	real dragCoeff = k1 * speed + k2 * speed * speed;
	force = v.unit() * -dragCoeff;

	particle->addForce(force);
}

ParticleField::ParticleField()
{}

ParticleField::ParticleField(const Vector2& s, real r, real k_p)
{
	source = s;
	radius = r;
	k = k_p;
}

void ParticleField::updateForce(Particle *particle, real duration)
{
	Vector2 v = particle->getPosition() - source;
	real d = v.magnitude();

	if (d == 0 || d > radius)
		return;

	Vector2 force = v.unit() * (k / (d*d));
	particle->addForce(force);
}

void ParticleField::setSource(const Vector2& s)
{
	source = s;
}

ParticleSpring::ParticleSpring(Particle *o, real sc, real rl)
{
	other = o;
	springConstant = sc;
	restLength = rl;
}

void ParticleSpring::updateForce(Particle *particle, real duration)
{
	Vector2 force;
	Vector2 v = particle->getPosition() - other->getPosition();

	// F = -k * (l - l0)
	force = v.unit() * -(v.magnitude() - restLength) * springConstant;
	particle->addForce(force);
}

ParticleAnchoredSpring::ParticleAnchoredSpring(Vector2 *a, real sc, real rl)
{
	anchor = a;
	springConstant = sc;
	restLength = rl;
}

void ParticleAnchoredSpring::updateForce(Particle *particle, real duration)
{
	Vector2 force;
	Vector2 v = particle->getPosition() - *anchor;

	// F = -k * (l - l0)
	force = v.unit() * -(v.magnitude() - restLength) * springConstant;
	particle->addForce(force);
}

ParticleBungee::ParticleBungee(Particle *o, real sc, real rl)
{
	other = o;
	springConstant = sc;
	restLength = rl;
}

void ParticleBungee::updateForce(Particle *particle, real duration)
{
	Vector2 force;
	Vector2 v = particle->getPosition() - other->getPosition();

	if (v.magnitude() <= restLength)
		return;

	// F = -k * (l - l0)
	force = v.unit() * -(v.magnitude() - restLength) * springConstant;
	particle->addForce(force);
}

ParticleAnchoredBungee::ParticleAnchoredBungee(Vector2 *a, real sc, real rl)
{
	anchor = a;
	springConstant = sc;
	restLength = rl;
}

void ParticleAnchoredBungee::updateForce(Particle *particle, real duration)
{
	Vector2 force;
	Vector2 v = particle->getPosition() - *anchor;

	if (v.magnitude() <= restLength)
		return;

	// F = -k * (l - l0)
	force = v.unit() * -(v.magnitude() - restLength) * springConstant;
	particle->addForce(force);
}

ParticleBuoyancy::ParticleBuoyancy(real md, real v, real wh, real ld)
{
	maxDepth = md;
	volume = v;
	waterHeight = wh;
	liquidDensity = ld;
}


void ParticleBuoyancy::updateForce(Particle *particle, real duration)
{
	real depth = particle->getPosition().y;
	Vector2 force(0, 0);

	if (depth >= waterHeight + maxDepth) // out of water
		;
	else if (depth <= waterHeight - maxDepth) // fully submerged
		force.y = liquidDensity * volume;
	else // partially submerged
		force.y = liquidDensity * volume 
			* (maxDepth - (depth - waterHeight)) / (2 * maxDepth);

	particle->addForce(force);
}

ParticleFakeSpring::ParticleFakeSpring(Vector2 *a, real sc, real d)
{
	anchor = a;
	springConstant = sc;
	damping = d;
}

// using damped harmonic motion to reversely calculate the required accelearation and force
void ParticleFakeSpring::updateForce(Particle *particle, real duration)
{
	if (!particle->isFiniteMass() || duration == 0)
		return;

	Vector2 p = particle->getPosition() - *anchor;
	real gamma = 0.5f * real_sqrt(4 * springConstant - damping * damping);
	if (gamma == 0)
		return;

	Vector2 c = p * (damping / (2 * gamma)) + particle->getVelocity() * (1.0f / gamma);
	Vector2 target = (p * real_cos(gamma * duration) + c * real_sin(gamma * duration))
		* real_exp(-0.5f * damping * duration);
	Vector2 force = ((target - p) * (1.0f / (duration * duration)) -
		particle->getVelocity() * (1.0f / duration)) * particle->getMass();

	particle->addForce(force);
}

ParticleControl::ParticleControl()
{}

ParticleControl::ParticleControl(Particle *p)
{
	other[0] = p;
	on = true;
}

ParticleControl::ParticleControl(Particle *other[N], Vector2 offset[N])
{
	for (int i = 0; i < N; i++)
	{
		this->other[i] = other[i];
		this->offset[i] = offset[i];
	}
	on = true;
}

void ParticleControl::switchOn(bool on)
{
	this->on = on;
}

void ParticleControl::updateForce(Particle *particle, real duration)
{
	if (!on)
		return;

	real k[2 * N] = { 1, 1.5 };

	Vector2 x[2 * N];
	for (int i = 0; i < N; i++)
	{
		x[i] = particle->getPosition() - (other[i]->getPosition() + offset[i]);
		x[N + i] = particle->getVelocity() - other[i]->getVelocity();
	}

	Vector2 kx(0, 0);
	for (int i = 0; i < 2 * N; i++)
		kx = kx + x[i] * k[i];

	Vector2 force = -kx * particle->getInverseMass();
	particle->addForce(force);
}


ParticlePointGravity::ParticlePointGravity(const Vector2& source, real G)
{
	this->source = source;
	this->G = G;
}

void ParticlePointGravity::updateForce(Particle *particle, real duration)
{
	Vector2 v = particle->getPosition() - source;
	real d = v.magnitude();

	if (d == 0)
		return;

	Vector2 force = v.unit() * -(G / (d*d));
	particle->addForce(force);
}