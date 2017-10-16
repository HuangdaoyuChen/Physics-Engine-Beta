#ifndef __BODY_H_INCLUDED__
#define __BODY_H_INCLUDED__


#include "precision.h"
#include "core.h"

class RigidBody
{
public:
	static real sleepEpsilon;

protected:
	Vector2 position; // position of center of mass
	Vector2 orientation;
	Vector2 velocity;
	real angularVelocity;
	Vector2 acceleration;
	Matrix3 transformMatrix; // only rotation and translation
	real inverseMass;
	real inverseMomentOfInertia;
	real linearDamping; // 0 to 1
	real angularDamping; // 0 to 1
	Vector2 forceAccum;
	real torqueAccum;

	real motion;
	bool isAwake;
	bool canSleep;

public:
	RigidBody();
	RigidBody(const Vector2 &position, const Vector2 &orientation,
		real inverseMass, real inverseMomentOfInertia);

	// accessors
	Vector2 getPosition() const;
	Vector2 getOrientation() const;
	Vector2 getVelocity() const;
	real getAngularVelocity() const;
	Vector2 getAcceleration() const;
	Matrix3 getTransformMatrix() const;
	real getInverseMass() const;
	real getMass() const;
	real getInverseMomentOfInertia() const;
	bool getIsAwake() const;

	Vector2 getPointInWorldSpace(const Vector2 &point);
	Vector2 getPointInLocalSpace(const Vector2 &point);

	void calculateDerivedData();
	void addForce(const Vector2 &force);
	void integrate(real duration);
	void clearAccumulators();
	// force is in world coord, point is in local coord
	void addForceAtBodyPoint(const Vector2 &force, const Vector2 &point);
	// both are in world coord
	void addForceAtPoint(const Vector2 &force, const Vector2 &point);
	void applyImpulseAtPoint(const Vector2& impulse, const Vector2 &point);
	bool isFiniteMass() const;
	void applyTorque(real torque);

	Vector2 getVelocityAtPoint(const Vector2 &point);
	void move(const Vector2& displacement);
	void rotate(real rotation);

	void setAwake(const bool awake = true);
	static void setSleepEpsilon(real sleepEpsilon);
};


#endif // __BODY_H_INCLUDED__