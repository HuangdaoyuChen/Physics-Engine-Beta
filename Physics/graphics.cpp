#include "graphics.h"

void drawAxis()
{
	glBegin(GL_LINES);
	{
		glVertex2f(0.0f, 1.0f);
		glVertex2f(0.0f, -1.0f);
		glVertex2f(1.0f, 0.0f);
		glVertex2f(-1.0f, 0.0f);
	}
	glEnd();
}

void drawMouse(Vector2 position, real radius)
{
	glPushMatrix();
	{
		glTranslatef(position.x, position.y, 0);
		glScalef(radius, radius, 1);
		drawCircle();
	}
	glPopMatrix();
}

// Draw a square
void drawSquare()
{
	// Draw the square
	glBegin(GL_LINE_LOOP);
	{
		glVertex2f(+1.0f, +1.0f);
		glVertex2f(-1.0f, +1.0f);
		glVertex2f(-1.0f, -1.0f);
		glVertex2f(+1.0f, -1.0f);
	}
	glEnd();
}

// draw a N-vertex polygon with the array of vertex coordinate <x,y>
void drawPolygon(float x[], float y[], const int N)
{
	glBegin(GL_POLYGON);
	for (int i = 0; i < N; i++)
		glVertex2f(x[i], y[i]);
	glEnd();
}

// draw a circle
void drawCircle()
{
	const int N = 50; // number of triangle fans
	const float d = (2 * PI) / N;

	glBegin(GL_LINE_LOOP);
	//glVertex2f(0, 0);
	for (int i = 0; i < N + 1; i++)
		glVertex2f(cos(i * d), sin(i * d));
	glEnd();
}

void drawVector2(const Vector2& v)
{
	glBegin(GL_LINES);
	{
		glVertex2f(0.0f, 0.0f);
		glVertex2f(v.x, v.y);
	}
	glEnd();
}

void drawLine(const Vector2& v1, const Vector2& v2)
{
	glBegin(GL_LINES);
	{
		glVertex2f(v1.x, v1.y);
		glVertex2f(v2.x, v2.y);
	}
	glEnd();
}

void drawParticle(Particle *p)
{
	glPushMatrix();
	{
		glTranslatef(p->getPosition().x, p->getPosition().y, 0);
		//drawVector2(p.getVelocity());
		glScalef(0.01, 0.01, 1);
		drawCircle();
	}
	glPopMatrix();
}

void drawRigidBody(RigidBody *body)
{
	if (!body->getIsAwake())
		glColor3fv(SECONDARY_COLOR);
	else
		glColor3fv(OBJECT_COLOR);

	float m[16];
	body->getTransformMatrix().fillGLMatrix(m);

	glPushMatrix();
	{
		glMultMatrixf(m);
		//drawVector2(Vector2(1, 0));
		glScalef(0.01, 0.01, 1);
		drawSquare();
	}
	glPopMatrix();
}

void drawSpring(RigidBody *body, Spring *spring)
{
	Vector2 c1 = body->getPointInWorldSpace(spring->connectionPoint);
	Vector2 c2 = spring->other->getPointInWorldSpace(spring->otherConnectionPoint);
	drawLine(c1, c2);
}

void drawContact(Contact *contact)
{
	glPushMatrix();
	{
		glTranslatef(contact->contactPoint.x, contact->contactPoint.y, 0);
		drawVector2(contact->contactNormal * -contact->penetration);
		glScalef(0.01, 0.01, 1);
		drawCircle();
	}
	glPopMatrix();
}

void drawCollisionData(CollisionData *data)
{
	for (int i = 0; i < data->contactsCount; i++)
		drawContact(&(data->contactArray[i]));
}

void drawCollisionSphere(CollisionSphere *sphere)
{
	float m[16];
	sphere->body->getTransformMatrix().fillGLMatrix(m);

	glPushMatrix();
	{
		glMultMatrixf(m);
		glScalef(sphere->radius, sphere->radius, 1);
		drawVector2(Vector2(1, 0));
		drawCircle();
	}
	glPopMatrix();
}

void drawCollisionBox(CollisionBox *box)
{
	float m[16];
	box->body->getTransformMatrix().fillGLMatrix(m);

	glPushMatrix();
	{
		glMultMatrixf(m);
		glScalef(box->halfSize.x, box->halfSize.y, 1);
		drawVector2(Vector2(1, 0));
		drawSquare();
	}
	glPopMatrix();
}

void drawCollisionPlane(CollisionPlane *plane)
{
	glPushMatrix();
	{
		Vector2 center = plane->normal * plane->offset;
		glTranslatef(center.x, center.y, 0);
		drawVector2(center.crossProduct(+4));
		drawVector2(center.crossProduct(-4));
	}
	glPopMatrix();
}

const int TRACEPOINT = 10;
int currentTrace = 0;
real trace[TRACEPOINT][2] = { 0 };

void drawTrace(Particle *p)
{
	trace[currentTrace][0] = p->getPosition().x;
	trace[currentTrace][1] = p->getPosition().y;
	currentTrace++;
	if (currentTrace >= TRACEPOINT)
		currentTrace = 0;

	glBegin(GL_POINTS);
	for (int i = 0; i < TRACEPOINT; i++)
		glVertex2f(trace[i][0], trace[i][1]);
	glEnd();
}

void drawParticleLink(ParticleLink *pl)
{
	drawLine(pl->particle[0]->getPosition(), pl->particle[1]->getPosition());
}

void drawField(Field *field)
{
	glPushMatrix();
	{
		glTranslatef(field->source.x, field->source.y, 0);
		glScalef(field->radius, field->radius, 1);
		drawCircle();
	}
	glPopMatrix();
}

void drawParticleField(ParticleField *field)
{
	glPushMatrix();
	{
		glTranslatef(field->source.x, field->source.y, 0);
		glScalef(field->radius, field->radius, 1);
		drawCircle();
	}
	glPopMatrix();
}

void drawJointAnchored(JointAnchored *jointAnchored)
{
	Vector2 a_pos_world = jointAnchored->body->getPosition();
	Vector2 b_pos_world = jointAnchored->position[1];

	drawLine(a_pos_world, b_pos_world);
}

void drawLink(Link *link)
{
	Vector2 a_pos_world = link->body[0]->getPosition();
	Vector2 b_pos_world = link->body[1]->getPosition();

	drawLine(a_pos_world, b_pos_world);
}