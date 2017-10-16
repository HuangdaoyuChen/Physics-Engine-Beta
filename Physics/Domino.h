#ifndef __DOMINO_H_INCLUDED__
#define __DOMINO_H_INCLUDED__

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


class DominoApp : public RigidBodyApplication
{
protected:
	static const int SPHERE_NUM = 1;
	static const int BOX_NUM = 10;
	static const int PLANE_NUM = 4;

	RigidBody sphere_bodies[SPHERE_NUM];
	RigidBody box_bodies[BOX_NUM];

	CollisionSphere spheres[SPHERE_NUM];
	CollisionBox boxes[BOX_NUM];
	CollisionPlane planes[PLANE_NUM];

protected:
	void generateContacts();

public:
	DominoApp();
	void display();
};


#endif