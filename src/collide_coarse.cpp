#include "collide_coarse.hpp"

using namespace cyclon;

template<typename BVC>
BVHNode<BVC>::BVHNode(BVHNode *parent, const BVC &volume, RigidBody *body)
    : parent(parent), volume(volume), body(body)
{
    children[0] = children[1] = nullptr;
}


template<typename BVC>
BVHNode<BVC>::~BVHNode()
{
    if (parent){
        BVHNode<BVC> *sibling;
        if (parent->children[0] == this) sibling=parent->children[1];
        else sibling=parent->children[0];

        parent->volume = sibling->volume;
        parent->body = sibling->body;
        parent->children[0] = sibling->children[0];
        parent->children[1] = sibling->children[1];
//        parent->children = sibling->children;
        sibling->parent = sibling->body = sibling->children[0] = sibling->children[1] = nullptr;
        delete sibling;

        parent->recalculateBoundingVolume();
    }

    if (children[0]){
        children[0]->parent = nullptr;
        delete children[0];
    }
    if (children[1]){
        children[1]->parent = nullptr;
        delete children[1];
    }
}

template<typename BVC>
bool BVHNode<BVC>::isLeaf() const
{
    return (body != nullptr);
}

template<typename BVC>
bool BVHNode<BVC>::overlaps(const BVHNode<BVC> *other) const
{
    return volume->overlaps(other->volume);
}

template<typename BVC>
unsigned BVHNode<BVC>::getPotentialContactsWith(BVHNode<BVC> *other,
                                                PotentialContact *contacts,
                                                unsigned limit) const
{
    if (!overlaps(other) || limit == 0) return 0;
    if (isLeaf() && other->isLeaf()){
        contacts->body[0] = body;
        contacts->body[1] = other->body;
        return 1;
    }

    if (other->isLeaf() || (
                !isLeaf() && volume->getSize() >= other->volume->getSize()
                ))
    {
        unsigned count = children[0]->getPotentialContactsWith(
                    other, contacts, limit);
        if (limit > count){
            return count + children[1]->getPotentialContactsWith(
                    other, contacts+count, limit-count);
        }
        else{
            return count;
        }
    }
    else{
        unsigned count = children[0]->getPotentialContactsWith(
                    other->children[0], contacts, limit);
        if (limit > count){
            return count + getPotentialContactsWith(
                    other->children[1], contacts+count, limit-count);
        }
        else{
            return count;
        }
    }
}

template<typename BVC>
unsigned BVHNode<BVC>::getPotentialContacts(PotentialContact *contacts, unsigned limit) const
{
    if (isLeaf() || limit == 0) return 0;
    children[0]->getPotentialContactsWith(children[1], contacts, limit);
}

template<typename BVC>
void BVHNode<BVC>::recalculateBoundingVolume(bool recurse)
{
    if (isLeaf()) return;

    volume = BVC{children[0]->volume, children[1]->volume};

    if (parent) parent->recalculateBoundingVolume(true);
}

template<typename BVC>
void BVHNode<BVC>::insert(RigidBody *newBody, const BVC &NewVolume)
{
    if(isLeaf()){
        children[0] = new BVHNode<BVC>{this, volume, body};
        children[1] = new BVHNode<BVC>{this, NewVolume, newBody};
        this->body = nullptr;
        recalculateBoundingVolume();
    }
    else{
        if(children[0]->volume.getGrowth(NewVolume) <
                children[1]->volume.getGrowth(NewVolume)){
            children[0]->insert(newBody, NewVolume);
        }
        else{
            children[1]->insert(newBody, NewVolume);
        }
    }
}

BoundingSphere::BoundingSphere(const Vector3 &center, real radius)
    :center(center), radius(radius)
{
}

BoundingSphere::BoundingSphere(const BoundingSphere &one, const BoundingSphere &two)
{
    Vector3 centreOffset = two.center - one.center;
    real distance = centreOffset.squareMagnitude();
    real radiusDiff = two.radius - one.radius;

    // Check if the larger sphere encloses the small one
    if (radiusDiff*radiusDiff >= distance)
    {
        if (one.radius > two.radius)
        {
            center = one.center;
            radius = one.radius;
        }
        else
        {
            center = two.center;
            radius = two.radius;
        }
    }

    // Otherwise we need to work with partially
    // overlapping spheres
    else
    {
        distance = real_sqrt(distance);
        radius = (distance + one.radius + two.radius) * ((real)0.5);

        // The new centre is based on one's centre, moved towards
        // two's centre by an ammount proportional to the spheres'
        // radii.
        center = one.center;
        if (distance > 0)
        {
            center += centreOffset * ((radius - one.radius)/distance);
        }
    }

}

real BoundingSphere::getGrowth(const BoundingSphere &other) const
{
    BoundingSphere newSphere(*this, other);

    // We return a value proportional to the change in surface
    // area of the sphere.
    return newSphere.radius*newSphere.radius - radius*radius;
}

bool BoundingSphere::overlaps(BoundingSphere *other) const
{
    real distanceSquared = (center - other->center).squareMagnitude();
    return distanceSquared < (radius+other->radius) * (radius+other->radius);
}
