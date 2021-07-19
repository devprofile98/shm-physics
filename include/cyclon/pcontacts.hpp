#ifndef CYCLON_PCONTACTS
#define CYCLON_PCONTACTS

#include "particle.hpp"

namespace cyclon {

    class ParticleContact{
    public:
        Particle* particles[2];
        real restitution;
        Vector3 contactNormal;

    protected:
        void resolve(real duration);
        real calculateSepratingVelocity();

    private:
        void resolveVelocity(real duration);
    };
}


#endif  //CYCLON_PCONTACTS
