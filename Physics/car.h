#ifndef __CAR_H_INCLUDED__
#define __CAR_H_INCLUDED__

#include <windows.h>
#include <GL/glut.h>
#include <iostream>

#include "precision.h"
#include "core.h"

#include "body.h"
#include "fgen.h"
#include "joints.h"
#include "world.h"
#include "collide_fine.h"

#include "graphics.h"
#include "app.h"


class CarApp : public RigidBodyApplication
{
protected:
	static const int SPHERE_NUM = 20;
	static const int BOX_NUM = 20;
	static const int PLANE_NUM = 4;
	static const int JOINT_NUM = 3;

	RigidBody sphere_bodies[SPHERE_NUM];
	RigidBody box_bodies[BOX_NUM];

	CollisionSphere spheres[SPHERE_NUM];
	CollisionBox boxes[BOX_NUM];
	CollisionPlane planes[PLANE_NUM];

	Joint joints[JOINT_NUM];

	Spring wheelSpring[2];
	Spring carSpring[2];

	bool isWheelOn;

protected:
	void generateContacts();

public:
	CarApp();
	virtual void updateForce(real duration);
	void display();

	void keyboard(unsigned char key);
};


#endif