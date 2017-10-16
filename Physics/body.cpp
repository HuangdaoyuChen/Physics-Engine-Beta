#include "body.h"

real RigidBody::sleepEpsilon = 0;

RigidBody::RigidBody()
	: orientation(1, 0)
{
	angularVelocity = 0;
	inverseMass = 1;
	inverseMomentOfInertia = 1;
	linearDamping = (real)0.99;
	angularDamping = (real)0.99;
	torqueAccum = 0;

	motion = 10 * sleepEpsilon;
	isAwake = false;
	canSleep = true;
	calculateDerivedData();
}

RigidBody::RigidBody(const Vector2 &position, const Vector2 &orientation,
	real inverseMass, real inverseMomentOfInertia)
	: orientation(1, 0)
{
	this->position = position;
	if (orientation.magnitude() > 0)
		this->orientation = orientation;
	this->inverseMass = inverseMass;
	this->inverseMomentOfInertia = inverseMomentOfInertia;
	angularVelocity = 0;
	linearDamping = (real)0.99;
	angularDamping = (real)0.99;
	torqueAccum = 0;

	motion = 10 * sleepEpsilon;
	isAwake = false;
	canSleep = true;
	calculateDerivedData();
}

Vector2 RigidBody::getPosition() const
{
	return position;
}

Vector2 RigidBody::getOrientation() const
{
	return orientation;
}

Vector2 RigidBody::getVelocity() const
{
	return velocity;
}

real RigidBody::getAngularVelocity() const
{
	return angularVelocity;
}

Vector2 RigidBody::getAcceleration() const
{
	return acceleration;
}

Matrix3 RigidBody::getTransformMatrix() const
{
	return transformMatrix;
}

real RigidBody::getInverseMass() const
{
	return inverseMass;
}

real RigidBody::getMass() const
{
	if (inverseMass == 0)
		return 0;

	return 1.0f / inverseMass;
}

real RigidBody::getInverseMomentOfInertia() const
{
	return inverseMomentOfInertia;
}

bool RigidBody::getIsAwake() const
{
	return isAwake;
}

void RigidBody::calculateDerivedData()
{
	orientation.normalize();
	transformMatrix.setOrientationAndPos(orientation, position);
}

void RigidBody::addForce(const Vector2 &force)
{
	forceAccum.add(force);
	isAwake = true;
}

void RigidBody::integrate(real duration)
{
	if (!isAwake)
		return;

	// a = f / m
	acceleration = forceAccum * inverseMass;
	real angularAcceleration = torqueAccum * inverseMomentOfInertia;
	// v = v + a*t
	velocity.addScaledVector(acceleration, duration);
	angularVelocity += angularAcceleration * duration;
	// v = v * d^t
	velocity.scale(real_pow(linearDamping, duration));
	angularVelocity *= real_pow(angularDamping, duration);
	// p = p + v*t     + (1/2 * a*t^2) ~ 0
	position.add(velocity * duration);
	position.add(acceleration * (duration * duration / 2));
	orientation.rotate(angularVelocity * duration);
	orientation.rotate(angularAcceleration * (duration * duration / 2));

	calculateDerivedData();
	clearAccumulators();

	if (canSleep)
	{
		const real baseBias = 0.5f;
		real currentMotion = velocity * velocity + angularVelocity * angularVelocity;
		real bias = real_pow(baseBias, duration);
		motion = bias * motion + (1 - bias) * currentMotion;

		if (motion < sleepEpsilon)
			setAwake(false);
		else if (motion > 10 * sleepEpsilon)
			motion = 10 * sleepEpsilon;
	}
}

void RigidBody::clearAccumulators()
{
	forceAccum.clear();
	torqueAccum = 0;
}

Vector2 RigidBody::getPointInWorldSpace(const Vector2 &point)
{
	return (transformMatrix * point);
}

Vector2 RigidBody::getPointInLocalSpace(const Vector2 &point)
{
	return (transformMatrix.transformInverse(point));
}

void RigidBody::addForceAtBodyPoint(const Vector2 &force, const Vector2 &point)
{
	Vector2 world = transformMatrix * point;
	addForceAtPoint(force, world);
	isAwake = true;
}

void RigidBody::addForceAtPoint(const Vector2 &force, const Vector2 &point)
{
	Vector2 arm = point - position;
	forceAccum.add(force);
	torqueAccum += arm.crossProduct(force);
	isAwake = true;
}

void RigidBody::applyImpulseAtPoint(const Vector2& impulse, const Vector2 &point)
{
	// linear
	Vector2 deltaLinearVelocity = impulse *  inverseMass;
	velocity.add(deltaLinearVelocity);
	// angular
	real deltaAngularVelocity = -(impulse.crossProduct(point - position)) * inverseMomentOfInertia;
	angularVelocity += deltaAngularVelocity;
	isAwake = true;
}

bool RigidBody::isFiniteMass() const
{
	return (inverseMass > 0);
}

void RigidBody::applyTorque(real torque)
{
	torqueAccum += torque;
	isAwake = true;
}

Vector2 RigidBody::getVelocityAtPoint(const Vector2 &point)
{
	// v = theta_dot.cross(r);
	Vector2 rotVelLocal = point.crossProduct(-point.magnitude() * angularVelocity);
	Vector2 rotVelWorld = transformMatrix.transformDirection(rotVelLocal);
	return velocity + rotVelWorld;
}

void RigidBody::move(const Vector2& displacement)
{
	position.add(displacement);
}

void RigidBody::rotate(real rotation)
{
	orientation.rotate(rotation);
}

void RigidBody::setAwake(const bool awake)
{
	
	if (awake)
	{
		isAwake = true;
		motion = sleepEpsilon * 2.0f;
	}
	else
	{
		isAwake = false;
		velocity.clear();
		angularVelocity = 0;
	}
}

void RigidBody::setSleepEpsilon(real sleepEpsilon)
{
	RigidBody::sleepEpsilon = sleepEpsilon;
}