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
    }

}
