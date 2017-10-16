#ifndef __CURTAIN_H_INCLUDED__
#define __CURTAIN_H_INCLUDED__

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


class CurtainApp : public RigidBodyApplication
{
protected:
	static const int ROW_NUM = 15;
	static const int SPHERE_NUM = ROW_NUM * ROW_NUM;
	static const int BOX_NUM = 1;
	static const int PLANE_NUM = 4;
	static const int ROD_NUM = 2 * (ROW_NUM - 1) * (ROW_NUM - 1) 
		+ 2 * (ROW_NUM - 1);

	RigidBody sphere_bodies[ROW_NUM][ROW_NUM];
	RigidBody box_bodies[BOX_NUM];

	CollisionSphere spheres[SPHERE_NUM];
	CollisionBox boxes[BOX_NUM];
	CollisionPlane planes[PLANE_NUM];

	Cable rod[ROD_NUM];
	JointAnchored jointAnchored[ROW_NUM];

protected:
	void generateContacts();

public:
	CurtainApp();
	void display();
	void keyboard(unsigned char key);
};


#endif