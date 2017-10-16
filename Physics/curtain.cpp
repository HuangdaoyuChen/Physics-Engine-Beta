#include "curtain.h"

CurtainApp::CurtainApp() :
RigidBodyApplication()
{
	collisionData.restitution = 0.5;
	gravity.setGravity(Vector2(0, -0.1));
	field.k = 0.01;

	// walls
	real wallDist = 0.9;
	planes[0] = CollisionPlane(Vector2::Y, -wallDist);
	planes[1] = CollisionPlane(-Vector2::Y, -wallDist);
	planes[2] = CollisionPlane(Vector2::X, -wallDist * 2);
	planes[3] = CollisionPlane(-Vector2::X, -wallDist * 2);

	// stack
	real sphereMass = 1;
	real sphereRadius = 0.01;
	real sphereMOI = 1;

	real boxMass = 10;
	Vector2 boxHalfSize(0.2, 0.2);
	real boxMOI = boxMass * boixMOIPerMass(boxHalfSize);

	real gap = 0.05;
	Vector2 sphereStackPosition(-0.0, 0.0);
	Vector2 sphereStackGapY(0, -gap);
	Vector2 sphereStackGapX(gap, 0);
	Vector2 boxStackPosition(-0.2, -wallDist + boxHalfSize.y);
	Vector2 boxStackGap(0, boxHalfSize.y * 2);

	for (int i = 0; i < ROW_NUM; i++)
		for (int j = 0; j < ROW_NUM; j++)
		{
			sphere_bodies[i][j] = RigidBody(sphereStackPosition
				+ sphereStackGapX * i + sphereStackGapY * j * 0.1,
				Vector2::X, 1.0f / sphereMass, 1.0f / sphereMOI);
		}

	for (int i = 0; i < BOX_NUM; i++)
	{
		box_bodies[i] = RigidBody(boxStackPosition + boxStackGap * i,
			Vector2(1, 0.0), 1.0f / boxMass, 1.0f / boxMOI);
	}

	for (int i = 0; i < ROW_NUM; i++)
		for (int j = 0; j < ROW_NUM; j++)
			spheres[i * ROW_NUM + j] = CollisionSphere(&sphere_bodies[i][j], sphereRadius);

	for (int i = 0; i < BOX_NUM; i++)
		boxes[i] = CollisionBox(&box_bodies[i], boxHalfSize);

	// joints
	real length = gap;
	real restitution = 0.5;
	for (int i = 0; i < ROW_NUM - 1; i++)
		for (int j = 0; j < ROW_NUM - 1; j++)
		{
			int rodIndex = i * (ROW_NUM - 1) + j;
			rod[rodIndex] = Cable(&sphere_bodies[i][j], Vector2::ORIGIN,
				&sphere_bodies[i + 1][j], Vector2::ORIGIN, gap, restitution);
			rod[rodIndex + (ROW_NUM - 1) * (ROW_NUM - 1)] = 
				Cable(&sphere_bodies[i][j], Vector2::ORIGIN,
				&sphere_bodies[i][j + 1], Vector2::ORIGIN, gap, restitution);
		}

	for (int i = 0; i < ROW_NUM - 1; i++)
	{
		int baseIndex = 2 * (ROW_NUM - 1) * (ROW_NUM - 1);
		rod[baseIndex + i] = Cable(&sphere_bodies[i][ROW_NUM - 1], Vector2::ORIGIN,
			&sphere_bodies[i + 1][ROW_NUM - 1], Vector2::ORIGIN, gap, restitution);
		rod[baseIndex + ROW_NUM - 1 + i] = Cable(&sphere_bodies[ROW_NUM - 1][i], Vector2::ORIGIN,
			&sphere_bodies[ROW_NUM - 1][i + 1], Vector2::ORIGIN, gap, restitution);
	}

	for (int i = 0; i < ROW_NUM; i++)
	{
		jointAnchored[i] = JointAnchored(&sphere_bodies[i][0], Vector2::ORIGIN,
			sphere_bodies[i][0].getPosition(), 0);
	}

	// assign to world
	for (int i = 0; i < ROW_NUM; i++)
		for (int j = 0; j < ROW_NUM; j++)
			world.getRigidBodies().push_back(&sphere_bodies[i][j]);
	for (int i = 0; i < BOX_NUM; i++)
		world.getRigidBodies().push_back(&box_bodies[i]);

	World::RigidBodies::iterator i = world.getRigidBodies().begin();
	for (; i != world.getRigidBodies().end(); i++)
	{
		world.getForceRegistry().add((*i), &gravity);
		world.getForceRegistry().add((*i), &field);
		world.getForceRegistry().add((*i), &aero);
	}

	for (int i = 0; i < ROD_NUM; i++)
		world.getContactGenerators().push_back(&rod[i]);
	for (int i = 0; i < ROW_NUM; i++)
		world.getContactGenerators().push_back(&jointAnchored[i]);
}

void CurtainApp::generateContacts()
{
	collisionData.reset();

	/*for (int i = 0; i < SPHERE_NUM; i++)
		for (int j = i + 1; j < SPHERE_NUM; j++)
			CollisionDetector::sphereAndSphere(spheres[i], spheres[j], &collisionData);*/

	for (int i = 0; i < BOX_NUM; i++)
		for (int j = i + 1; j < BOX_NUM; j++)
			CollisionDetector::boxAndBox2(boxes[i], boxes[j], &collisionData);

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

void CurtainApp::display()
{
	/*World::RigidBodies::iterator i = world.getRigidBodies().begin();
	for (; i != world.getRigidBodies().end(); i++)
		drawRigidBody(*i);*/

	glColor3fv(OBJECT_COLOR);
	//for (int i = 0; i < SPHERE_NUM; i++)
	//	drawCollisionSphere(&spheres[i]);
	for (int i = 0; i < BOX_NUM; i++)
		drawCollisionBox(&boxes[i]);
	for (int i = 0; i < PLANE_NUM; i++)
		drawCollisionPlane(&planes[i]);
	for (int i = 0; i < ROD_NUM; i++)
		drawLink(&rod[i]);

	glColor3fv(SECONDARY_COLOR);
	drawField(&field);

	//drawCollisionData(&collisionData);
}

void CurtainApp::keyboard(unsigned char key)
{
	switch (key)
	{
	case 'g':
		gravity.setOn(!gravity.on);
		break;

	case ' ':
		for (int i = 0; i < ROW_NUM ; i++)
			for (int j = 0; j < ROW_NUM; j++)
				sphere_bodies[i][j].applyImpulseAtPoint(Vector2(0.1, 0), Vector2(0, 0));
		break;

	default:
		break;
	}
}