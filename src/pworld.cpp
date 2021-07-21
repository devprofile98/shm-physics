#include "pworld.hpp"

namespace cyclon {
    ParticleWorld::ParticleWorld(uint32_t maxContacts, uint32_t iterations)
        :resolver(iterations), maxContacts(maxContacts)
    {
        contacts = new ParticleContact[maxContacts];
        calculateIterations = (iterations == 0);
    }

    ParticleWorld::~ParticleWorld()
    {
        delete[] contacts;
    }

void ParticleWorld::startFrame(){

         for(Particles::iterator p = particles.begin();
             p!=particles.end();
             p++
             )
         {
             (*p)->clearAccumulator();
         }

}

    uint32_t ParticleWorld::generateContacts()
    {
       uint32_t limit = maxContacts;
       ParticleContact *nextContact = contacts;
       for(ContactGenerators::iterator g =  contactGenerators.begin();
           g!=contactGenerators.end();
           g++
           )
       {

           unsigned used =(*g)->addContact(nextContact, limit);
           limit -= used;
           nextContact += used;

           // We've run out of contacts to fill. This means we're missing
           // contacts.
           if (limit <= 0) break;
       }

       // Return the number of cont
       return maxContacts - limit;
    }

    void ParticleWorld::integrate(real duration)
    {
        for(Particles::iterator p =  particles.begin();
            p != particles.end();
            p++
            )
        {
            (*p)->integrate(duration);
        }

    }

    void ParticleWorld::runPhysics(real duration)
    {
        registry.updateForces(duration);
        integrate(duration);
        uint32_t usedContact = generateContacts();
        if (calculateIterations) resolver.setIterations(usedContact * 2);
        resolver.resolveContacts(contacts, usedContact, duration);
    }
}

