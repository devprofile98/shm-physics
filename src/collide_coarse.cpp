#include "collide_coarse.hpp"

using namespace cyclon;

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

bool BoundingSphere::overlaps(BoundingSphere *other) const
{
    real distanceSquared = (center - other->center).squareMagnitude();
    return distanceSquared < (radius+other->radius) * (radius+other->radius);
}
