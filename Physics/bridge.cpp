#include "bridge.h"

BridgeApp::BridgeApp() :
RigidBodyApplication()
{
	// walls
	real wallDist = 0.9;
	planes[0] = CollisionPlane(Vector2::Y, -wallDist);
	planes[1] = CollisionPlane(-Vector2::Y, -wallDist);
	planes[2] = CollisionPlane(Vector2::X, -wallDist * 2);
	planes[3] = CollisionPlane(-Vector2::X, -wallDist * 2);

	// stack
	real sphereMass = 5;
	real sphereRadius = 0.1;
	real sphereMOI = sphereMass * sphereMOIPerMass(sphereRadius);

	real boxMass = 1;
	Vector2 boxHalfSize(0.1, 0.02);
	real boxMOI = boxMass * boixMOIPerMass(boxHalfSize);

	Vector2 sphereStackPosition(0, 0.5);
	Vector2 sphereStackGap(0, sphereRadius * 2);
	Vector2 boxStackPosition(-1, 0);
	Vector2 boxStackGap(boxHalfSize.x * 2, 0);

	for (int i = 0; i < SPHERE_NUM; i++)
	{
		sphere_bodies[i] = RigidBody(sphereStackPosition + sphereStackGap * i,
			Vector2::X, 1.0f / sphereMass, 1.0f / sphereMOI);
	}

	for (int i = 0; i < BOX_NUM; i++)
	{
		box_bodies[i] = RigidBody(boxStackPosition + boxStackGap * i,
			Vector2(1, 0.0), 1.0f / boxMass, 1.0f / boxMOI);
	}

	for (int i = 0; i < SPHERE_NUM; i++)
		spheres[i] = CollisionSphere(&sphere_bodies[i], sphereRadius);

	for (int i = 0; i < BOX_NUM; i++)
		boxes[i] = CollisionBox(&box_bodies[i], boxHalfSize);

	// joints
	real error = 0.03;
	Vector2 jointPosition(boxHalfSize.x, 0);
	for (int i = 0; i < JOINT_NUM; i++)
		joints[i] = Joint(&box_bodies[i], jointPosition,
			&box_bodies[i + 1], -jointPosition, error);
	jointsAnchored[0] = JointAnchored(&box_bodies[0], Vector2::ORIGIN,
		box_bodies[0].getPosition(), 0);
	jointsAnchored[1] = JointAnchored(&box_bodies[BOX_NUM-1], Vector2::ORIGIN,
		box_bodies[BOX_NUM-1].getPosition(), 0);

	// assign to world
	for (int i = 0; i < SPHERE_NUM; i++)
		world.getRigidBodies().push_back(&sphere_bodies[i]);
	for (int i = 0; i < BOX_NUM; i++)
		world.getRigidBodies().push_back(&box_bodies[i]);

	World::RigidBodies::iterator i = world.getRigidBodies().begin();
	for (; i != world.getRigidBodies().end(); i++)
	{
		world.getForceRegistry().add((*i), &gravity);
		world.getForceRegistry().add((*i), &field);
		world.getForceRegistry().add((*i), &aero);
	}
	
	for (int i = 0; i < JOINT_NUM; i++)
		world.getContactGenerators().push_back(&joints[i]);

	for (int i = 0; i < JOINTANCHORED_NUM; i++)
		world.getContactGenerators().push_back(&jointsAnchored[i]);
}

void BridgeApp::generateContacts()
{
	collisionData.reset();

	for (int i = 0; i < SPHERE_NUM; i++)
		for (int j = i + 1; j < SPHERE_NUM; j++)
			CollisionDetector::sphereAndSphere(spheres[i], spheres[j], &collisionData);

	/*for (int i = 0; i < BOX_NUM; i++)
		for (int j = i + 1; j < BOX_NUM; j++)
			CollisionDetector::boxAndBox2(boxes[i], boxes[j], &collisionData);*/

	for (int i = 0; i < BOX_NUM; i++)
		for (int j = 0; j < SPHERE_NUM; j++)
			CollisionDetector::boxAndSphere(boxes[i], spheres[j], &collisionData);

	for (int i = 0; i < PLANE_NUM; i++)
	{
		for (int j = 0; j < SPHERE_NUM; j++)
			CollisionDetector::sphereAndHalfSpace(spheres[j], planes[i], &collisionData);
		for (int j = 0; j < BOX_NUM; j++)
			CollisionDetector::boxAndHalfSpace(boxes[j], planes[i], &collisionData);
	}
}

void BridgeApp::display()
{
	World::RigidBodies::iterator i = world.getRigidBodies().begin();
	for (; i != world.getRigidBodies().end(); i++)
		drawRigidBody(*i);

	glColor3fv(OBJECT_COLOR);
	for (int i = 0; i < SPHERE_NUM; i++)
		drawCollisionSphere(&spheres[i]);
	for (int i = 0; i < BOX_NUM; i++)
		drawCollisionBox(&boxes[i]);
	for (int i = 0; i < PLANE_NUM; i++)
		drawCollisionPlane(&planes[i]);

	glColor3fv(SECONDARY_COLOR);
	drawField(&field);

	drawCollisionData(&collisionData);
}