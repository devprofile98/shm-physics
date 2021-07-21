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

    real Particle::getInverseMass()
    {
        return inverseMass;
    }

    Vector3 Particle::getPosition()
    {
        return  this->position;
    }

    Vector3 Particle::getVelocity()
    {
        return this->velocity;
    }

    void Particle::getPosition(Vector3 *position) const
    {
        *position = this->position;
    }

    void Particle::setVelocity(const Vector3 &velocity)
    {
        this->velocity = velocity;
    }

    void Particle::setPosition(const Vector3 &position)
    {
        this->position = position;
    }


}
