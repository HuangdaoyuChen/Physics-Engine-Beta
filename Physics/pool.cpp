#include "pool.h"

PoolApp::PoolApp() :
RigidBodyApplication()
{
	collisionData.restitution = 0.9;
	collisionData.friction = 0.5;
	gravity.setOn(false);

	// wall
	real wallDist = 0.9;
	planes[0] = CollisionPlane(Vector2::Y, -wallDist);
	planes[1] = CollisionPlane(-Vector2::Y, -wallDist);
	planes[2] = CollisionPlane(Vector2::X, -wallDist * 2);
	planes[3] = CollisionPlane(-Vector2::X, -wallDist * 2);

	// stack
	real sphereMass = 1;
	real sphereRadius = 0.06;
	real sphereMOI = sphereMass * sphereMOIPerMass(sphereRadius);

	Vector2 position[16];
	real dist = real_sqrt(3) * sphereRadius;
	position[0] = Vector2(-1, 0);
	position[1] = Vector2(0.7, 0);
	position[2] = Vector2(position[1].x + dist, position[1].y - sphereRadius);
	position[3] = Vector2(position[1].x + dist, position[1].y + sphereRadius);
	position[4] = Vector2(position[2].x + dist, position[2].y - sphereRadius);
	position[5] = Vector2(position[3].x + dist, position[3].y - sphereRadius);
	position[6] = Vector2(position[3].x + dist, position[3].y + sphereRadius);
	position[7] = Vector2(position[4].x + dist, position[4].y - sphereRadius);
	position[8] = Vector2(position[5].x + dist, position[5].y - sphereRadius);
	position[9] = Vector2(position[6].x + dist, position[6].y - sphereRadius);
	position[10] = Vector2(position[6].x + dist, position[6].y + sphereRadius);
	position[11] = Vector2(position[7].x + dist, position[7].y - sphereRadius);
	position[12] = Vector2(position[8].x + dist, position[8].y - sphereRadius);
	position[13] = Vector2(position[9].x + dist, position[9].y - sphereRadius);
	position[14] = Vector2(position[10].x + dist, position[10].y - sphereRadius);
	position[15] = Vector2(position[10].x + dist, position[10].y + sphereRadius);

	for (int i = 0; i < SPHERE_NUM; i++)
	{
		sphere_bodies[i] = RigidBody(position[i],
			Vector2::X, 1.0f / sphereMass, 1.0f / sphereMOI);
	}

	for (int i = 0; i < SPHERE_NUM; i++)
		spheres[i] = CollisionSphere(&sphere_bodies[i], sphereRadius);


	// assign to world
	for (int i = 0; i < SPHERE_NUM; i++)
		world.getRigidBodies().push_back(&sphere_bodies[i]);

	World::RigidBodies::iterator i = world.getRigidBodies().begin();
	for (; i != world.getRigidBodies().end(); i++)
	{
		world.getForceRegistry().add((*i), &gravity);
		world.getForceRegistry().add((*i), &field);
		world.getForceRegistry().add((*i), &aero);
	}
}

void PoolApp::generateContacts()
{
	collisionData.reset();

	for (int i = 0; i < SPHERE_NUM; i++)
		for (int j = i + 1; j < SPHERE_NUM; j++)
			CollisionDetector::sphereAndSphere(spheres[i], spheres[j], &collisionData);

	for (int i = 0; i < PLANE_NUM; i++)
	{
		for (int j = 0; j < SPHERE_NUM; j++)
			CollisionDetector::sphereAndHalfSpace(spheres[j], planes[i], &collisionData);
	}
}

void PoolApp::display()
{
	World::RigidBodies::iterator i = world.getRigidBodies().begin();
	for (; i != world.getRigidBodies().end(); i++)
		drawRigidBody(*i);

	glColor3fv(OBJECT_COLOR);
	for (int i = 0; i < SPHERE_NUM; i++)
		drawCollisionSphere(&spheres[i]);
	for (int i = 0; i < PLANE_NUM; i++)
		drawCollisionPlane(&planes[i]);

	glColor3fv(SECONDARY_COLOR);
	drawField(&field);

	drawCollisionData(&collisionData);
}

void PoolApp::keyboard(unsigned char key)
{
	switch (key)
	{
	case 'g':
		gravity.setOn(!gravity.on);
		break;

	case ' ':
		sphere_bodies[0].applyImpulseAtPoint(Vector2(10, 0), Vector2(0, 0));
		break;

	default:
		break;
	}
}