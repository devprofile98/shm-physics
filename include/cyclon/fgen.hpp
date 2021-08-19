#ifndef CYCLON_FGEN_H
#define CYCLON_FGEN_H

#include "body.hpp"
#include "pfgen.hpp"
#include <vector>

namespace cyclon {

    // base class for all kind of force generators
    class ForceGenerator{
    public:
        virtual void updateForce(RigidBody *body, real duration) = 0;
    };

    class Gravity: public ForceGenerator{
        Vector3 gravity;
    public:
        Gravity(const Vector3& gravity);
        virtual void updateForce(RigidBody *body, real duration) override;
    };


    class Spring: ForceGenerator{
        Vector3 connectionPoint, otherConnectionPoint;
        RigidBody* other;
        real springConstant;
        real restLength;

    public:
        Spring(const Vector3& localConnectionPt,
               RigidBody* other,
               const Vector3& otherConnectionPt,
               real springLength,
               real restLength
               );

        virtual void updateForce(RigidBody *body, real duration) override;

    };
}

#endif //CYCLON_FGEN_H
