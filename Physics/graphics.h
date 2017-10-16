#ifndef __GRAPHICS_H_INCLUDED__
#define __GRAPHICS_H_INCLUDED__


#include <windows.h>
#include <GL/glut.h>
#include <iostream>

#include "precision.h"
#include "core.h"

#include "particle.h"
#include "pfgen.h"
#include "plinks.h"
#include "pcontacts.h"
#include "pworld.h"

#include "body.h"
#include "fgen.h"
#include "joints.h"
#include "world.h"
#include "collide_fine.h"

const GLfloat gray = 0.8f;
const GLfloat lightGray = 0.2f;
const GLfloat backGround = 0.1f;
const GLfloat OBJECT_COLOR[3] = { gray, gray, gray };
const GLfloat SECONDARY_COLOR[3] = { lightGray, lightGray, 1.0f };

void drawAxis();
void drawMouse(Vector2 position, real radius);
void drawCircle();
void drawSquare();
void drawPolygon(float x[], float y[], const int N);
void drawVector2(const Vector2& v);
void drawLine(const Vector2& v1, const Vector2& v2);
void drawParticle(Particle *p);
void drawRigidBody(RigidBody *body);
void drawSpring(RigidBody *body, Spring *spring);
void drawContact(Contact *contact);
void drawCollisionData(CollisionData *data);
void drawCollisionSphere(CollisionSphere *sphere);
void drawCollisionBox(CollisionBox *box);
void drawCollisionPlane(CollisionPlane *plane);
void drawTrace(Particle *p);
void drawParticleLink(ParticleLink *pl);
void drawField(Field *field);
void drawParticleField(ParticleField *field);
void drawJointAnchored(JointAnchored *jointAnchored);
void drawLink(Link *link);


#endif // __GRAPHICS_H_INCLUDED__