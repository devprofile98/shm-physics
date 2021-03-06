#ifndef CYCLON_PARTICLE
#define CYCLON_PARTICLE

// particle is the simplest object that can be
// simulated in a physics system

#include "core.hpp"

namespace cyclon {
    class Particle{
    public:

        real damping;
        real inverseMass;

        // keep track of each particle property
        Vector3 position, velocity, acceleration;

        //accumulated force to be applied in next iteration, zeroed each integration step
        Vector3 forceAccum;

        // integrator definition
        void integrate(real duration);
        void clearAccumulator();
        void addForce(const Vector3& force);

        bool hasFiniteMass();
        real getMass();
        real getInverseMass();
        Vector3 getPosition();
        Vector3 getVelocity();
        void getPosition(Vector3 *position) const ;

        void setVelocity(const Vector3& velocity);
        void setPosition(const Vector3& position);
    };
}
#endif //CYCLON_PARTICLE
