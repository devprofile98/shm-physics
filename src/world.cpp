#include "world.hpp"

namespace cyclon {

void World::startFrame()
{
    BodyRegistration *reg = firstBody;
    while (reg) {
        reg->body->clearAccumulators();
        reg->body->calculateDerivedData();

        reg = reg->next;
    }
}

void World::runPhysics(real duration)
{
    BodyRegistration *firstReg = firstBody;
    while (firstReg) {
        firstReg->body->integrate(duration);
        firstReg = firstReg->next;
    }
}

}
