#ifndef __POOL_H_INCLUDED__
#define __POOL_H_INCLUDED__

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


class PoolApp : public RigidBodyApplication
{
protected:
	static const int SPHERE_NUM = 16;
	static const int PLANE_NUM = 4;

	RigidBody sphere_bodies[SPHERE_NUM];

	CollisionSphere spheres[SPHERE_NUM];
	CollisionPlane planes[PLANE_NUM];

protected:
	void generateContacts();

public:
	PoolApp();
	void display();

	void keyboard(unsigned char key);
};


#endif