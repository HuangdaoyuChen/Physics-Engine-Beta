#ifndef __APP_H_INCLUDED__
#define __APP_H_INCLUDED__

#include <windows.h>
#include <GL/glut.h>
#include <iostream>

#include "precision.h"
#include "core.h"

#include "particle.h"
#include "pfgen.h"
#include "plinks.h"
#include "pcontacts.h"
#include "pworld.h"

#include "body.h"
#include "fgen.h"
#include "joints.h"
#include "world.h"
#include "collide_fine.h"

#include "graphics.h"


class ParticleApplication
{
public:
	static const int PARTICLE_NUM = 10;
	bool gravityOn;
	Vector2 g;

	ParticleWorld world;
	Particle particles[PARTICLE_NUM];

	ParticleGravity gravity;
	ParticleDrag drag;
	ParticleField field;

	ParticleControl control[PARTICLE_NUM];

	ParticleRod rod;

public:
	ParticleApplication();
	void update(real duration);
	void display();

	void setFieldSource(const Vector2& position);
	void switchGravity();
};


class RigidBodyApplication
{
protected:
	static const int MAX_CONTACT = 10000;
	static const int ITERATION = 4;

	World world;

	Gravity gravity;
	Aero aero;
	Field field;

	CollisionData collisionData;
	ContactResolver resolver;

protected:
	virtual void generateContacts() = 0;

public:
	RigidBodyApplication();
	virtual void updateForce(real duration);
	virtual void update(real duration);
	virtual void display() = 0;

	virtual void passiveMotion(const Vector2& position);
	virtual void keyboard(unsigned char key);

	static real sphereMOIPerMass(real radius);
	static real boixMOIPerMass(const Vector2& halfsize);
};


#endif // __APP_H_INCLUDED__