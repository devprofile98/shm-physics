#ifndef CYCLON_WORLD_H
#define CYCLON_WORLD_H

#include "body.hpp"

namespace cyclon {
class World{

    struct BodyRegistration{
        RigidBody* body;
        BodyRegistration* next;
    };

    BodyRegistration* firstBody;
public:
    void startFrame();
    void runPhysics(real duration);


};
}


#endif //CYCLON_WORLD_H
