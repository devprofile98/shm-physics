#ifndef CYCLON_PLINK
#define CYCLON_PLINK

#include <cstdint>
#include "pcontacts.hpp"
#include "particle.hpp"

namespace cyclon {

    class ParticleLink{
    public:
        Particle *particle[2];
        virtual uint32_t fillContact(ParticleContact *contact, uint32_t limit) const = 0;


    protected:
        real currentLength() const;
    };


    class ParticleCable: public ParticleLink{
        real maxLength;
        real restitution;

        virtual uint32_t fillContact(ParticleContact *contact, uint32_t limit) const override;
    };


    class ParticleRod: public ParticleLink{
    public:
        real length;
        real currentLength() const;
        virtual uint32_t fillContact(ParticleContact *contact, uint32_t limit) const override;
    };
}





#endif //CYCLON_PLINK
