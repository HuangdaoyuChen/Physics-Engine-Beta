#include "contacts.h"

void Contact::setBodyData(RigidBody* body1, RigidBody* body2,
	real friction, real restitution)
{
	this->body[0] = body1;
	this->body[1] = body2;
	this->friction = friction;
	this->restitution = restitution;
}

void Contact::calculateInternals(real duration)
{
	contactNormal.normalize();
	if (body[0] == NULL)
		swapBodies();

	calculateContactBasis();

	relativeContactPosition[0] = contactPoint - body[0]->getPosition();
	if (body[1] != NULL)
		relativeContactPosition[1] = contactPoint - body[1]->getPosition();

	contactVelocity = calculateLocalVelocity(0, duration);
	if (body[1] != NULL)
		contactVelocity.minus(calculateLocalVelocity(1, duration));

	calculateDesiredDeltaVelocity(duration);
}

void Contact::swapBodies()
{
	contactNormal.invert();
	RigidBody* temp = body[0];
	body[0] = body[1];
	body[1] = temp;
}

void Contact::calculateContactBasis()
{
	Vector2 tangent = contactNormal.crossProduct(-1);
	contactToWorld.setComponents(contactNormal, tangent);
}

void Contact::calculateDesiredDeltaVelocity(real duration)
{
	const static real velocityLimit = (real)0.01f;

	real velocityFromAcc = 0;
	velocityFromAcc = body[0]->getAcceleration() * duration * contactNormal;

	if (body[1] != NULL)
		velocityFromAcc -= body[1]->getAcceleration() * duration * contactNormal;

	real thisRestitution = restitution;
	if (real_abs(contactVelocity.x) < velocityLimit)
		thisRestitution = 0;
	
	desiredDeltaVelocity = -contactVelocity.x - (contactVelocity.x - velocityFromAcc) * thisRestitution;
}

Vector2 Contact::calculateLocalVelocity(int bodyIndex, real duration)
{
	RigidBody *thisBody = body[bodyIndex];
	Vector2 velocityRot = relativeContactPosition[bodyIndex].crossProduct
		(-thisBody->getAngularVelocity());
	Vector2 velocityNet = velocityRot + thisBody->getVelocity();
	Vector2 contactVelocity = contactToWorld.transpose() * velocityNet;

	Vector2 accVelocity = thisBody->getAcceleration() * duration;
	accVelocity = contactToWorld.transpose() * accVelocity;
	accVelocity.x = 0;
	contactVelocity.add(accVelocity);
	return contactVelocity;
}

Vector2 Contact::calculateFrictionLessImpulse()
{
	real deltaVelLocal = 0;
	for (int i = 0; i < 2; i++)
	{
		if (body[i] != NULL)
		{
			Vector2 unitImpulse = contactNormal;
			real angularImpulse = relativeContactPosition[i].crossProduct(unitImpulse);
			real deltaAngularVelocity = angularImpulse * body[i]->getInverseMomentOfInertia();
			Vector2 deltaVelRotWorld = relativeContactPosition[i].crossProduct(-deltaAngularVelocity);
			deltaVelLocal += deltaVelRotWorld * contactNormal + body[i]->getInverseMass();
		}
	}

	Vector2 impulseContact;
	impulseContact.x = desiredDeltaVelocity / deltaVelLocal;
	impulseContact.y = 0;
	return impulseContact;
}

Vector2 Contact::calculateFrictionImpulse()
{
	Matrix2 sum_deltaVelocity(0, 0, 0, 0);
	for (int i = 0; i < 2; i++)
	{
		if (body[i] != NULL)
		{
			// [T^-1(-1/MOI * R*R)T + I*1/m] p = v
			// r % v % r = R v
			Vector2 r = relativeContactPosition[i];
			real IMOI = body[i]->getInverseMomentOfInertia();
			Matrix2 deltaVelocity(r.y * r.y * IMOI, -r.x * r.y * IMOI,
				-r.x * r.y * IMOI, r.x * r.x * IMOI);
			deltaVelocity = contactToWorld.transpose() * deltaVelocity * contactToWorld;
			real inverseMass = body[i]->getInverseMass();
			deltaVelocity = deltaVelocity + Matrix2(inverseMass, 0, 0, inverseMass);
			sum_deltaVelocity = sum_deltaVelocity + deltaVelocity;
		}
	}
	Matrix2 impulseMatrix = sum_deltaVelocity.inverse();
	Vector2 velKill(desiredDeltaVelocity, -contactVelocity.y);
	Vector2 impulseContact = impulseMatrix * velKill;

	if (real_abs(impulseContact.y) > friction * impulseContact.x)
	{ // dynamic friction
		impulseContact.y = impulseContact.y / real_abs(impulseContact.y);
		impulseContact.x = sum_deltaVelocity.data[0] +
			sum_deltaVelocity.data[1] * friction * impulseContact.y;
		impulseContact.x = desiredDeltaVelocity / impulseContact.x;
		impulseContact.y *= friction * impulseContact.x;
	}

	return impulseContact;
}

// todo: get velocityChange
void Contact::applyVelocityChange()
{
	if (contactVelocity.x >= 0)
		return;

	Vector2 impulse = contactToWorld * calculateFrictionImpulse();
	body[0]->applyImpulseAtPoint(impulse, contactPoint);

	velocityChange[0] = impulse *  body[0]->getInverseMass();
	rotationChange[0] = -(impulse.crossProduct(contactPoint - body[0]->getPosition()))
		* body[0]->getInverseMomentOfInertia();

	if (body[1] != NULL)
	{
		impulse.invert();
		body[1]->applyImpulseAtPoint(impulse, contactPoint);

		velocityChange[1] = impulse *  body[1]->getInverseMass();
		rotationChange[1] = -(impulse.crossProduct(contactPoint - body[1]->getPosition()))
			* body[1]->getInverseMomentOfInertia();
	}
}

void Contact::applyPositionChange()
{
	real linearMove[2];
	real angularMove[2];
	real totalInertia = 0;
	real linearInertia[2];
	real angularInertia[2];
	real deltaAngularVelocity[2];

	for (int i = 0; i < 2; i++)
		if (body[i] != NULL)

		{
			real angularImpulse = relativeContactPosition[i].crossProduct(contactNormal);
			deltaAngularVelocity[i] = angularImpulse * body[i]->getInverseMomentOfInertia();
			Vector2 deltaVelRotWorld = relativeContactPosition[i].crossProduct(-deltaAngularVelocity[i]);

			angularInertia[i] = deltaVelRotWorld * contactNormal;
			linearInertia[i] = body[i]->getInverseMass();
			totalInertia += angularInertia[i] + linearInertia[i];
		}

	real angularLimitConstant = 0.2f;
	// apply displacement and rotation
	for (int i = 0; i < 2; i++)
		if (body[i] != NULL)
		{
			int sign = 1 - i * 2;
			linearMove[i] = sign * penetration * (linearInertia[i] / totalInertia);
			angularMove[i] = sign * penetration * (angularInertia[i] / totalInertia);

			real limit = angularLimitConstant * relativeContactPosition[i].magnitude();
			if (real_abs(angularMove[i]) > limit)
			{
				real totalMove = linearMove[i] + angularMove[i];
				if (angularMove[i] >= 0)
					angularMove[i] = limit;
				else
					angularMove[i] = -limit;
				linearMove[i] = totalMove - angularMove[i];
			}

			linearChange[i] = contactNormal * linearMove[i];
			if (angularMove[i] == 0 || angularInertia[i] == 0)
				angularChange[i] = 0;
			else
				angularChange[i] = deltaAngularVelocity[i] / angularInertia[i] * angularMove[i];

			body[i]->move(linearChange[i]);
			body[i]->rotate(angularChange[i]);
			body[i]->calculateDerivedData();
		}
}

ContactResolver::ContactResolver(int velocityIteration, int positionIteration,
	real velocityEpsilon, real positionEpsilon)
{
	this->positionIteration = positionIteration;
	this->velocityIteration = velocityIteration;
	this->positionEpsilon = positionEpsilon;
	this->velocityEpsilon = velocityEpsilon;
}

void ContactResolver::setIterations(int velocityIteration, int positionIteration)
{
	ContactResolver::velocityIteration = velocityIteration;
	ContactResolver::positionIteration = positionIteration;
}

void ContactResolver::resolveContacts(Contact *contactArray,
	int numContacts, real duration)
{
	if (numContacts == 0)
		return;
	prepareContacts(contactArray, numContacts, duration);
	adjustPositions(contactArray, numContacts, duration);
	adjustVelocities(contactArray, numContacts, duration);
	
	/*for (int i = 0; i < numContacts; i++)
	{
		contactArray[i].calculateInternals(duration);
		contactArray[i].applyVelocityChange();
		contactArray[i].applyPositionChange();
	}*/
}

void ContactResolver::prepareContacts(Contact *contactArray,
	int numContacts, real duration)
{
	for (int i = 0; i < numContacts; i++)
		contactArray[i].calculateInternals(duration);
}

void ContactResolver::adjustPositions(Contact *contactArray,
	int numContacts, real duration)
{
	Vector2 linearChange[2];
	real angularChange[2];

	positionIterationUsed = 0;
	while (positionIterationUsed < positionIteration)
	{
		real max = positionEpsilon;
		int indexMax = -1;

		for (int i = 0; i < numContacts; i++)
		{
			if (contactArray[i].penetration > max)
			{
				max = contactArray[i].penetration;
				indexMax = i;
			}
		}
		if (indexMax == -1)
			break;

		contactArray[indexMax].applyPositionChange();
		contactArray[indexMax].calculateInternals(duration);

		// resolve penetration
		linearChange[0] = contactArray[indexMax].linearChange[0];
		linearChange[1] = contactArray[indexMax].linearChange[1];
		angularChange[0] = contactArray[indexMax].angularChange[0];
		angularChange[1] = contactArray[indexMax].angularChange[1];

		for (int i = 0; i < numContacts; i++)
		{
			for (int b = 0; b < 2; b++)
			{
				if (contactArray[i].body[b] != NULL)
				{
					for (int d = 0; d < 2; d++)
					{
						if (contactArray[i].body[b] == contactArray[indexMax].body[d])
						{
							Vector2 deltaPosition = linearChange[d] + 
								contactArray[i].relativeContactPosition[b].crossProduct(
								-angularChange[d]);

							int sign;
							if (b == 0)
								sign = -1;
							else
								sign = 1;

							contactArray[i].penetration +=
								deltaPosition * (contactArray[i].contactNormal) * sign;

							contactArray[i].calculateInternals(duration);
						}
					}
				}
			}
		}
		positionIterationUsed++;
	}
}

void ContactResolver::adjustVelocities(Contact *contactArray,
	int numContacts, real duration)
{
	Vector2 velocityChange[2];
	real rotationChange[2];

	velocityIterationUsed = 0;
	while (velocityIterationUsed < velocityIteration)
	{
		real max = velocityEpsilon;
		int indexMax = -1;

		for (int i = 0; i < numContacts; i++)
		{
			if (contactArray[i].desiredDeltaVelocity > max)
			{
				max = contactArray[i].desiredDeltaVelocity;
				indexMax = i;
			}
		}
		if (indexMax == -1)
			break;
		
		contactArray[indexMax].matchAwakeState();
		contactArray[indexMax].applyVelocityChange();
		contactArray[indexMax].calculateInternals(duration);

		// resolve penetration
		velocityChange[0] = contactArray[indexMax].velocityChange[0];
		velocityChange[1] = contactArray[indexMax].velocityChange[1];
		rotationChange[0] = contactArray[indexMax].rotationChange[0];
		rotationChange[1] = contactArray[indexMax].rotationChange[1];

		for (int i = 0; i < numContacts; i++)
		{
			for (int b = 0; b < 2; b++)
			{
				if (contactArray[i].body[b] != NULL)
				{
					for (int d = 0; d < 2; d++)
					{
						if (contactArray[i].body[b] == contactArray[indexMax].body[d])
						{
							Vector2 deltaVelocity = velocityChange[d] +
								contactArray[i].relativeContactPosition[b].crossProduct(
								-rotationChange[d]);

							int sign;
							if (b == 0)
								sign = 1;
							else
								sign = -1;

							contactArray[i].contactVelocity.add(
								contactArray[i].contactToWorld.transpose() * deltaVelocity * sign);

							//contactArray[i].calculateDesiredDeltaVelocity(duration);
							contactArray[i].calculateInternals(duration);
						}
					}
				}
			}
		}
		velocityIterationUsed++;
	}
}

void Contact::matchAwakeState()
{
	if (body[1] == NULL)
		return;

	bool isAwake0 = body[0]->getIsAwake();
	bool isAwake1 = body[1]->getIsAwake();

	if (isAwake0 && !isAwake1)
		body[1]->setAwake();
	else if (!isAwake0 && isAwake1)
		body[0]->setAwake();
}