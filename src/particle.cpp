#include "assert.h"
#include "particle.hpp"


namespace cyclon {

    void Particle::integrate(real duration)
    {
        assert(duration > 0.0);


        // update linear position
        position.addScaledVector(velocity, duration);

        // workout accelration from the force
        Vector3 resultingAcc = acceleration;
        resultingAcc.addScaledVector(forceAccum, inverseMass);

        // update velocity
        velocity.addScaledVector(resultingAcc, duration);

        // impose drag
        velocity *= real_pow(damping, duration);

        // clear accumulator
        clearAccumulator();
    }

    void Particle::clearAccumulator(){
        forceAccum.clear();
    }

    void Particle::addForce(const Vector3 &force){
        forceAccum += force;
    }

    bool Particle::hasFiniteMass()
    {
        return inverseMass >= 0.0f;
    }

    real Particle::getMass()
    {
        if (inverseMass == 0)
            return 1000000; // some very big mass
        else{
            return ((real)1.0)/inverseMass;
        }
    }

}
