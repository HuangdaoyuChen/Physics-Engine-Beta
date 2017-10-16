#include "car.h"

CarApp::CarApp() :
RigidBodyApplication()
{
	bool isWheelOn = false;

	// wall
	real wallDist = 0.9;
	planes[0] = CollisionPlane(Vector2::Y, -wallDist);
	planes[1] = CollisionPlane(-Vector2::Y, -wallDist);
	planes[2] = CollisionPlane(Vector2::X, -wallDist * 2);
	planes[3] = CollisionPlane(-Vector2::X, -wallDist * 2);

	// car
	Vector2 carPosition(-1.2, -0.5);
	real wheelMass = 1;
	real wheelRadius = 0.10;
	real wheelMOI = wheelMass * sphereMOIPerMass(wheelRadius);
	real carMass = 10;
	Vector2 carHalfSize(0.4, 0.1);
	Vector2 carTopHalfSize(0.2, 0.1);
	real carMOI = carMass * boixMOIPerMass(carHalfSize);
	Vector2 carWheelOffset1(+0.25, -0.1);
	Vector2 carWheelOffset2(-0.25, -0.1);
	Vector2 carTopOffset(-0.1, 0.2);

	sphere_bodies[0] = RigidBody(carPosition + carWheelOffset1,
		Vector2::X, 1.0f / wheelMass, 1.0f / wheelMOI);
	spheres[0] = CollisionSphere(&sphere_bodies[0], wheelRadius);
	sphere_bodies[1] = RigidBody(carPosition + carWheelOffset2,
		Vector2::X, 1.0f / wheelMass, 1.0f / wheelMOI);
	spheres[1] = CollisionSphere(&sphere_bodies[1], wheelRadius);

	box_bodies[0] = RigidBody(carPosition, 
		Vector2::X, 1.0f / carMass, 1.0f / carMOI);
	boxes[0] = CollisionBox(&box_bodies[0], carHalfSize);
	box_bodies[1] = RigidBody(carPosition + carTopOffset,
		Vector2::X, 1.0f / carMass, 1.0f / carMOI);
	boxes[1] = CollisionBox(&box_bodies[1], carTopHalfSize);

	real jointError = 0.05;
	joints[0] = Joint(&sphere_bodies[0], Vector2(0.0, 0.0),
		&box_bodies[0], carWheelOffset1, jointError);
	joints[1] = Joint(&sphere_bodies[1], Vector2(0.0, 0.0),
		&box_bodies[0], carWheelOffset2, jointError);
	joints[2] = Joint(&box_bodies[1], Vector2(0.0, 0.0),
		&box_bodies[0], carTopOffset, 0.0);

	real k = 1000;
	real c = 2 * real_sqrt((wheelMass + carMass) * k) * 0.8;
	wheelSpring[0] = Spring(Vector2::ORIGIN, &box_bodies[0], carWheelOffset1, k, c, 0.0);
	carSpring[0] = Spring(carWheelOffset1, &sphere_bodies[0], Vector2::ORIGIN, k, c, 0.0);
	wheelSpring[1] = Spring(Vector2::ORIGIN, &box_bodies[0], carWheelOffset2, k, c, 0.0);
	carSpring[1] = Spring(carWheelOffset2, &sphere_bodies[1], Vector2::ORIGIN, k, c, 0.0);

	// stack
	real sphereMass = 1;
	real sphereRadius = 0.05;
	real sphereMOI = sphereMass * sphereMOIPerMass(sphereRadius);
	real boxMass = 1;
	Vector2 boxHalfSize(0.05, 0.05);
	real boxMOI = boxMass * boixMOIPerMass(boxHalfSize);

	Vector2 sphereStackPosition(0.8, -wallDist + sphereRadius);
	Vector2 boxStackPosition(1.0, -wallDist + boxHalfSize.y);

	for (int i = 2; i < SPHERE_NUM; i++)
	{
		sphere_bodies[i] = RigidBody(sphereStackPosition + Vector2(0, (real)(i - 2) * sphereRadius * 2),
			Vector2::X, 1.0f / sphereMass, 1.0f / sphereMOI);
	}
	for (int i = 2; i < BOX_NUM; i++)
	{
		box_bodies[i] = RigidBody(boxStackPosition + Vector2(0, (real)(i - 2) * boxHalfSize.y * 2),
			Vector2(1, 0.0), 1.0f / boxMass, 1.0f / boxMOI);
	}

	for (int i = 2; i < SPHERE_NUM; i++)
		spheres[i] = CollisionSphere(&sphere_bodies[i], sphereRadius);

	for (int i = 2; i < BOX_NUM; i++)
		boxes[i] = CollisionBox(&box_bodies[i], boxHalfSize);
	
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

	world.getForceRegistry().add(&sphere_bodies[0], &wheelSpring[0]);
	world.getForceRegistry().add(&sphere_bodies[1], &wheelSpring[1]);
	world.getForceRegistry().add(&box_bodies[0], &carSpring[0]);
	world.getForceRegistry().add(&box_bodies[0], &carSpring[1]);

	for (int i = 0; i < JOINT_NUM; i++)
		world.getContactGenerators().push_back(&joints[i]);
}

void CarApp::generateContacts()
{
	collisionData.reset();

	for (int i = 0; i < SPHERE_NUM; i++)
		for (int j = i + 1; j < SPHERE_NUM; j++)
			CollisionDetector::sphereAndSphere(spheres[i], spheres[j], &collisionData);

	for (int i = 0; i < BOX_NUM; i++)
		for (int j = i + 1; j < BOX_NUM; j++)
			CollisionDetector::boxAndBox2(boxes[i], boxes[j], &collisionData);

	for (int i = 0; i < BOX_NUM; i++)
		for (int j = 2; j < SPHERE_NUM; j++)
			CollisionDetector::boxAndSphere(boxes[i], spheres[j], &collisionData);

	for (int i = 0; i < PLANE_NUM; i++)
	{
		collisionData.friction = 10;
		for (int j = 0; j < SPHERE_NUM; j++)
			CollisionDetector::sphereAndHalfSpace(spheres[j], planes[i], &collisionData);
		collisionData.friction = 0.5;
		for (int j = 0; j < BOX_NUM; j++)
			CollisionDetector::boxAndHalfSpace(boxes[j], planes[i], &collisionData);
	}
}

void CarApp::updateForce(real duration)
{
	if (isWheelOn)
		sphere_bodies[1].applyTorque(-1);
}

void CarApp::display()
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

void CarApp::keyboard(unsigned char key)
{
	switch (key)
	{
	case 'g':
		gravity.setOn(!gravity.on);
		break;

	case ' ':
		isWheelOn = !isWheelOn;
		break;

	default:
		break;
	}
}