#include "pfgen.hpp"

using namespace cyclon;


void ParticleForceRegistry::add(Particle *particle, ParticleForceGenerator *fg)
{
    registration.push_back({particle, fg});
}

void ParticleForceRegistry::remove(Particle *particle, ParticleForceGenerator *fg)
{

}

void ParticleForceRegistry::clear()
{

}

void ParticleForceRegistry::updateForces(real duration){
    Registry::iterator i = registration.begin();
    for(; i!=registration.end(); i++){
        i->fg->updateForce(i->particle, duration);
    }
}

ParticleGravity::ParticleGravity(const Vector3 &gravity)
    :gravity(gravity)
{

}

void ParticleGravity::updateForce(Particle *particle, real duration)
{
    if (!particle->hasFiniteMass()) return;
    particle->addForce(gravity * particle->getMass());
}


void ParticleSpring::updateForce(Particle *particle, real duration){
    Vector3 force;

    particle->getPosition(&force);
    force -= other->getPosition();

    real magnitude = force.magnitude();
    magnitude = real_abs(magnitude) - restLength;
    magnitude *= springConstant;

    force.normalize();
    force *= -magnitude;
    particle->addForce(force);
}

void ParticleAnchorSpring::updateForce(Particle *particle, real duration){
    Vector3 force;
    particle->getPosition(&force);
    force -= *anchor;

    real magnitude = force.magnitude();
    magnitude = real_abs(magnitude) - restLength;
    magnitude *= springConstant;

    force.normalize();
    force *= -magnitude;
    particle->addForce(force);
}

void ParticleBungee::updateForce(Particle *particle, real duration)
{
    Vector3 force;
    particle->getPosition(&force);
    force -= other->getPosition();

    real magnitude = force.magnitude();
    if (magnitude <= restLength) return;
    magnitude = real_abs(magnitude) - restLength;
    magnitude *= springConstant;

    force.normalize();
    force *= -magnitude;
    particle->addForce(force);
}
