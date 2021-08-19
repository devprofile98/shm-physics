#include "fgen.hpp"

namespace cyclon {

Gravity::Gravity(const cyclon::Vector3 &gravity)
    : gravity(gravity)
{

}

void Gravity::updateForce(RigidBody *body, real duration)
{
    if(!body->hasFiniteMass()){
        return;
    }

    body->addForce(gravity * body->getMass());

}

Spring::Spring(const Vector3 &localConnectionPt,
               RigidBody *other,
               const Vector3 &otherConnectionPt,
               real springConstant,
               real restLength)
    :connectionPoint(localConnectionPt), otherConnectionPoint(otherConnectionPt),
      other(other), springConstant(springConstant)
{
    this->restLength = restLength;
}

void Spring::updateForce(RigidBody *body, real duration)
{
    Vector3 lws = body->getPointInWorldSpace(connectionPoint);
    Vector3 ows = other->getPointInWorldSpace(otherConnectionPoint);

    Vector3 force = lws - ows;
    real magnitude = force.magnitude();
    magnitude = real_abs(magnitude - restLength);
    magnitude *=springConstant;

    force.normalize();
    force *= -magnitude;
    body->addForceAtPoint(force, lws);

}

}
