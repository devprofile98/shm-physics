#include "pcontacts.hpp"


namespace cyclon {

    void ParticleContact::resolve(real duration){
        resolveVelocity(duration);
    }

    real ParticleContact::calculateSepratingVelocity(){
        Vector3 relativeVelocity = particles[0]->getVelocity();
        if (particles[1]) relativeVelocity -= particles[1]->getVelocity();
        return relativeVelocity.scalarProduct(contactNormal);
    }

    void ParticleContact::resolveVelocity(real duration){
        real sepratingVelocity = calculateSepratingVelocity();
        if (sepratingVelocity > 0) return; // contact seprating or stationary, no impulse needed

        real newSepVelocity = -sepratingVelocity * restitution;
        real deltaVelocity = newSepVelocity - sepratingVelocity;

        real totalInverseMass = particles[0]->getInverseMass();
        if (particles[1]) totalInverseMass +=particles[1]->getInverseMass();

        if (totalInverseMass <= 0){
            return;
        }
        real impulse = deltaVelocity / totalInverseMass;
        Vector3 impulseVec = contactNormal * impulse;

        particles[0]->setVelocity(particles[0]->getVelocity() + (impulseVec * particles[0]->getInverseMass()));
        if (particles[1]){
            particles[1]->setVelocity(particles[1]->getVelocity() + (impulseVec * particles[1]->getInverseMass()));
        }
    }
}
