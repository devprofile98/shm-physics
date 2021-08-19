#ifndef CYCLON_BODY_H
#define CYCLON_BODY_H

#include "core.hpp"

namespace cyclon {

class RigidBody{
public:
    real inverseMass;
    bool isAwake;
    Vector3 position;
    Quaternion orientation;
    Vector3 velocity; // linear velocity
    Vector3 rotation; // angular velocity
    Vector3 forceAccum;
    Vector3 torqueAccum;


    Matrix3 inverseInertiaTensor;
    Matrix3 inverseInertiaTensorWorld;
    Matrix4 transformMatrix;

    void calculateDerivedData();
    void setInertiaTensor(const Matrix3& inertiaTensor);
    Vector3 getPointInWorldSpace(const Vector3 &point) const;
    void addForceAtBodyPoint(const Vector3& force,
                             const Vector3& point);
    bool hasFiniteMass() const;
    real getMass() const;

    void addForceAtPoint(const Vector3 &force,
                                    const Vector3 &point);

    void addForce(const Vector3& force);
    void clearAccumulators();
    void integrate(real duration);

};

}
#endif //CYCLON_BODY_H
