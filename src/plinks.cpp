#include "plinks.hpp"

namespace cyclon {
    real ParticleLink::currentLength() const{
        Vector3 relativePos = particle[0]->getPosition() -
                              particle[1]->getPosition();
        return relativePos.magnitude();
    }

    uint32_t ParticleCable::fillContact(ParticleContact *contact, uint32_t limit) const{
        real length = currentLength();
        if (length < maxLength){
            return 0;
        }

        contact->particles[0] = particle[0];
        contact->particles[1] = particle[1];

        Vector3 normal = particle[1]->getPosition() - particle[0]->getPosition();
        normal.normalize();
        contact->contactNormal = normal;

        contact->pentration = length - maxLength;
        contact->restitution = restitution;
        return 1;
    }

    real ParticleRod::currentLength() const
    {
        Vector3 relativePos = particle[0]->getPosition() -
                              particle[1]->getPosition();
        return relativePos.magnitude();
    }

    uint32_t ParticleRod::fillContact(ParticleContact *contact, uint32_t limit) const
    {
        real currentLen = currentLength();
        if (currentLen == length) return 0;

        contact->particles[0] = particle[0];
        contact->particles[1] = particle[1];

        Vector3 normal = particle[1]->getPosition() - particle[0]->getPosition();
        normal.normalize();
        contact->contactNormal = normal;

        if (currentLen > length){
            contact->contactNormal = normal;
            contact->pentration = currentLen - length;
        }
        else{
            contact->contactNormal = normal * -1;
            contact->pentration = length - currentLen;
        }
        contact->restitution = 0;
        return 1;
    }


}
