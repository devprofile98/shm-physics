#ifndef CYCLON_PWORLD_H
#define CYCLON_PWORLD_H

#include <vector>
#include "particle.hpp"
#include "pfgen.hpp"
#include "plinks.hpp"

namespace cyclon {
    class ParticleWorld{
    public:
        typedef std::vector<Particle*> Particles;
        typedef std::vector<ParticleContactGenerator*> ContactGenerators;

        ParticleWorld(uint32_t maxContacts, uint32_t iterations=0);
        ~ParticleWorld();
        void startFrame();
        uint32_t generateContacts();
        void integrate(real duration);
        void runPhysics(real duration);

    private:

        Particles particles; // hold a list of particles
        ParticleForceRegistry registry;
        ParticleContactResolver resolver;

        ContactGenerators contactGenerators;
        ParticleContact *contacts;
        uint32_t maxContacts;
        bool calculateIterations;

    };
}

#endif //CYCLON_PWORLD_H
