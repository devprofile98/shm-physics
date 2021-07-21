#ifndef CYCLON_PCONTACTS
#define CYCLON_PCONTACTS

#include "particle.hpp"
#include <cstdint>

namespace cyclon {

    class ParticleContact{
    public:
        Particle* particles[2];
        real restitution;
        real pentration;
        Vector3 contactNormal;
        real calculateSepratingVelocity();
        void resolve(real duration);
        void resolveInterpentration(real duration);
    protected:

    private:
        void resolveVelocity(real duration);
    };

    class ParticleContactResolver{

    public:
        ParticleContactResolver(uint32_t iterations);
        void setIterations(uint32_t iterations);
        void resolveContacts(ParticleContact *contactArray, uint32_t numContact, real duration);

    protected:
        uint32_t iterations;
        uint32_t iterationUsed;
    };

    class ParticleContactGenerator{
    public:
        virtual uint32_t addContact(ParticleContact *contact, uint32_t limit) const =0;
    };
}


#endif  //CYCLON_PCONTACTS
