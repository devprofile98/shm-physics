#include "pfgen.hpp"

using namespace cyclon;

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
