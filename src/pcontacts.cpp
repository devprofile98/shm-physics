#include "pcontacts.hpp"


namespace cyclon {

    void ParticleContact::resolve(real duration){
        resolveVelocity(duration);
        resolveInterpentration(duration);
    }

    void ParticleContact::resolveInterpentration(real duration)
    {
        if (pentration <=0) return;

        real totalInverseMass = particles[0]->getInverseMass();
        if (particles[1]) totalInverseMass += particles[1]->getInverseMass();

        if (totalInverseMass <=0) return;

        Vector3 movePerIMass = contactNormal * (-pentration/totalInverseMass);
        particles[0]->setPosition(particles[0]->getPosition() +
                                  movePerIMass * particles[0]->getInverseMass()
                        );

        if (particles[1]){
            particles[1]->setPosition(particles[1]->getPosition() +
                                      movePerIMass * particles[1]->getInverseMass()
                            );
        }
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

    ParticleContactResolver::ParticleContactResolver(uint32_t iterations)
    {
        this->iterations = iterations;
    }

    void ParticleContactResolver::setIterations(uint32_t iterations)
    {
        ParticleContactResolver::iterations = iterations;
    }

    void ParticleContactResolver::resolveContacts(ParticleContact *contactArray, uint32_t numContact, real duration)
    {
        uint32_t iterationUsed = 0;
        real max = 0;
        uint32_t maxIndex = numContact;
        while(iterationUsed < iterations){
            max = 0;
            for (uint32_t index=0; index < numContact; index++){
                real sepVel = contactArray[index].calculateSepratingVelocity();

                if (max > sepVel){
                  max = sepVel;
                  maxIndex = index;

                }
            }
            contactArray[maxIndex].resolve(duration);
            iterationUsed++;
        }
    }

}
