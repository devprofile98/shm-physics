#ifndef CYCLON_COLLIDE_COARSE_H
#define CYCLON_COLLIDE_COARSE_H

#include "body.hpp"

namespace cyclon {

struct BoundingSphere{
    Vector3 center;
    real radius;

    BoundingSphere(const Vector3& center, real radius);
    BoundingSphere(const BoundingSphere &one, const BoundingSphere &two);

    bool overlaps(BoundingSphere *other) const;
};



struct PotentialContact{
    RigidBody* body[2];
};

template <typename BVC>
class BVHNode{
public:
    BVHNode *children[2];
    BVC volume;

    RigidBody *body;

    bool isLeaf() const;
    bool overlaps(const BVHNode<BVC> *other) const;

    unsigned getPotentialContactsWith(
        BVHNode<BVC> *other,
        PotentialContact* contacts,
        unsigned limit) const;

    unsigned getPotentialContacts(PotentialContact* contacts,
                                 unsigned limit) const;
};

}

#endif //CYCLON_COLLIDE_COARSE_H
