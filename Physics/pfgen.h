#ifndef __PFGEN_H_INCLUDED__
#define __PFGEN_H_INCLUDED__


#include <vector>
#include "precision.h"
#include "core.h"
#include "particle.h"

class ParticleForceGenerator
{
public:
	virtual void updateForce(Particle *particle, real duration) = 0;
};

class ParticleForceRegistry
{
protected:
	struct ParticleForceRegistration
	{
		Particle *particle;
		ParticleForceGenerator *fg;
	};

	typedef std::vector<ParticleForceRegistration> Registry;
	Registry registrations;

public:
	void add(Particle *particle, ParticleForceGenerator *fg);
	void remove(Particle *particle, ParticleForceGenerator *fg);
	void clear();
	void updateForces(real duration);
};

// gravity
class ParticleGravity : public ParticleForceGenerator
{
private:
	Vector2 gravity;

public:
	ParticleGravity();
	ParticleGravity(const Vector2& g);
	void setGravity(const Vector2& gravity);
	virtual void updateForce(Particle *particle, real duration);
};

// drag
class ParticleDrag : public ParticleForceGenerator
{
private:
	real k1;
	real k2;

public:
	ParticleDrag();
	ParticleDrag(real k1, real k2);
	virtual void updateForce(Particle *particle, real duration);
};

// force field
class ParticleField : public ParticleForceGenerator
{
public:
	Vector2 source;
	real radius;
	real k;

public:
	ParticleField();
	ParticleField(const Vector2& s, real r, real k);
	virtual void updateForce(Particle *particle, real duration);
	void setSource(const Vector2& s);
};


// spring
class ParticleSpring : public ParticleForceGenerator
{
private:
	Particle *other;
	real springConstant;
	real restLength;

public:
	ParticleSpring(Particle *o, real sc, real rl);
	virtual void updateForce(Particle *particle, real duration);
};

// anchored spring
class ParticleAnchoredSpring : public ParticleForceGenerator
{
private:
	Vector2 *anchor;
	real springConstant;
	real restLength;

public:
	ParticleAnchoredSpring(Vector2 *a, real sc, real rl);
	virtual void updateForce(Particle *particle, real duration);
};

// bungee
class ParticleBungee : public ParticleForceGenerator
{
private:
	Particle *other;
	real springConstant;
	real restLength;

public:
	ParticleBungee(Particle *o, real sc, real rl);
	virtual void updateForce(Particle *particle, real duration);
};

// anchored bungee
class ParticleAnchoredBungee : public ParticleForceGenerator
{
private:
	Vector2 *anchor;
	real springConstant;
	real restLength;

public:
	ParticleAnchoredBungee(Vector2 *a, real sc, real rl);
	virtual void updateForce(Particle *particle, real duration);
};

// buoyancy
class ParticleBuoyancy : public ParticleForceGenerator
{
private:
	real maxDepth;
	real volume;
	real waterHeight;
	real liquidDensity;

public:
	ParticleBuoyancy(real md, real v, real wh, real ld);
	virtual void updateForce(Particle *particle, real duration);
};

// fake damped spring
class ParticleFakeSpring : public ParticleForceGenerator
{
private:
	Vector2 *anchor;
	real springConstant;
	real damping;

public:
	ParticleFakeSpring(Vector2 *a, real sc, real d);
	virtual void updateForce(Particle *particle, real duration);
};

class ParticleControl : public ParticleForceGenerator
{
public:
	static const int N = 1;
	bool on;

private:
	Particle* other[N];
	Vector2 offset[N];
	
public:
	ParticleControl();
	ParticleControl(Particle *p);
	ParticleControl(Particle *other[N], Vector2 offset[N]);
	void switchOn(bool on);
	virtual void updateForce(Particle *particle, real duration);
};

class ParticlePointGravity : public ParticleForceGenerator
{
private:
	Vector2 source;
	real G;

public:
	ParticlePointGravity(const Vector2& source, real G);
	virtual void updateForce(Particle *particle, real duration);
};


#endif // __PFGEN_H_INCLUDED__