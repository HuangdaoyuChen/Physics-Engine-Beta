#include "app.h"

ParticleApplication::ParticleApplication() :
gravityOn(false),
g(0, -0.2),
world(100, 0),
gravity(Vector2::ORIGIN),
drag(0.2, 0.2),
field(Vector2(100, 0), 0.2, 0.01)
{
	particles[0] = Particle(Vector2(0.5, 0.2), 1);
	particles[1] = Particle(Vector2(0.5, 0.0), 1);

	for (int i = 2; i < PARTICLE_NUM; i++)
		particles[i] = Particle(Vector2((real)i / 10 - 1.0, -0.2), 1);

	rod = ParticleRod(&(particles[0]), &(particles[1]), 0.2);

	for (int i = 0; i < PARTICLE_NUM-1; i++)
		control[i] = ParticleControl(&particles[i+1]);
	control[PARTICLE_NUM - 1] = ParticleControl(&particles[0]);

	// assign to world
	for (int i = 0; i < PARTICLE_NUM; i++)
		world.getParticles().push_back(&particles[i]);

	ParticleWorld::Particles::iterator i = world.getParticles().begin();
	for (; i != world.getParticles().end(); i++)
	{
		world.getForceRegistry().add((*i), &gravity);
		world.getForceRegistry().add((*i), &drag);
		world.getForceRegistry().add((*i), &field);
	}

	for (int i = 0; i < PARTICLE_NUM; i++)
		world.getForceRegistry().add(&particles[i], &control[i]);

	//world.getContactGenerators().push_back(&rod);
}

void ParticleApplication::update(real duration)
{
	world.startFrame();
	world.runPhysics(duration);
}

void ParticleApplication::display()
{
	glColor3fv(OBJECT_COLOR);
	ParticleWorld::Particles::iterator i = world.getParticles().begin();
	for (; i != world.getParticles().end(); i++)
		drawParticle(*i);

	glColor3fv(SECONDARY_COLOR);
	drawParticleLink(&rod);
	drawParticleField(&field);
}

void ParticleApplication::switchGravity()
{
	gravityOn = !gravityOn;
	if (gravityOn)
		gravity.setGravity(g);
	else
		gravity.setGravity(Vector2::ORIGIN);
}

void ParticleApplication::setFieldSource(const Vector2& position)
{
	field.setSource(position);
}


RigidBodyApplication::RigidBodyApplication() :
world(MAX_CONTACT, 0),
gravity(Vector2(0, -0.4), true),
aero(Matrix2(-0.1, 0, 0, -0.1), Vector2::ORIGIN, &Vector2::ORIGIN),
field(Vector2(100, 0), 0.2, 0.05, Vector2(0.0, 0.0)),
collisionData(MAX_CONTACT, 0.4, 0.5),
resolver(MAX_CONTACT, MAX_CONTACT, -0.0, -0.0)
{
	RigidBody::setSleepEpsilon(0.001);
	collisionData.reset();
}

void RigidBodyApplication::updateForce(real duration)
{}

void RigidBodyApplication::update(real duration)
{
	for (int i = 0; i < ITERATION; i++)
	{
		world.startFrame();
		updateForce(duration / ITERATION);
		world.runPhysics(duration / ITERATION);

		generateContacts();
		resolver.setIterations(collisionData.contactsCount * 2, collisionData.contactsCount * 2);
		resolver.resolveContacts(collisionData.contactArray,
			collisionData.contactsCount, duration);
		//std::cout << collisionData.contactsCount << "\n";
	}
}

void RigidBodyApplication::keyboard(unsigned char key)
{
	switch (key)
	{
	case 'g':
		gravity.setOn(!gravity.on);
		break;

	default:
		break;
	}
}

void RigidBodyApplication::passiveMotion(const Vector2& position)
{
	field.setSource(position);
}

real RigidBodyApplication::sphereMOIPerMass(real radius)
{
	return 2.0 / 5 * radius * radius;
}

real RigidBodyApplication::boixMOIPerMass(const Vector2& halfsize)
{
	return 1.0 / 3 *(halfsize.x * halfsize.x + halfsize.y * halfsize.y);
}

