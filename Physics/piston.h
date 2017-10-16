#ifndef __PISTON_H_INCLUDED__
#define __PISTON_H_INCLUDED__

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


class PistonApp : public RigidBodyApplication
{
protected:
	static const int SPHERE_NUM = 1;
	static const int BOX_NUM = 2;
	static const int PLANE_NUM = 4;
	static const int JOINT_NUM = 2;
	static const int JOINTANCHORED_NUM = 1;

	RigidBody sphere_bodies[SPHERE_NUM];
	RigidBody box_bodies[BOX_NUM];

	CollisionSphere spheres[SPHERE_NUM];
	CollisionBox boxes[BOX_NUM];
	CollisionPlane planes[PLANE_NUM];

	Joint joints[JOINT_NUM];
	JointAnchored jointsAnchored[JOINT_NUM];

	bool isPistonOn;

protected:
	void generateContacts();

public:
	PistonApp();
	void display();

	virtual void updateForce(real duration);
	void keyboard(unsigned char key);
};


#endif