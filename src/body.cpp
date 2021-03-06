#include "body.hpp"

namespace cyclon {

static inline void _calculateTransformMatrix(
        Matrix4 &transformMatrix,
        const Vector3 &position,
        const Quaternion &orientation
        )
{
    transformMatrix.data[0] = 1-2*orientation.j*orientation.j-
            2*orientation.k*orientation.k;
    transformMatrix.data[1] = 2*orientation.i*orientation.j -
            2*orientation.r*orientation.k;
    transformMatrix.data[2] = 2*orientation.i*orientation.k +
            2*orientation.r*orientation.j;
    transformMatrix.data[3] = position.x;

    transformMatrix.data[4] = 2*orientation.i*orientation.j +
            2*orientation.r*orientation.k;
    transformMatrix.data[5] = 1-2*orientation.i*orientation.i-
            2*orientation.k*orientation.k;
    transformMatrix.data[6] = 2*orientation.j*orientation.k -
            2*orientation.r*orientation.i;
    transformMatrix.data[7] = position.y;

    transformMatrix.data[8] = 2*orientation.i*orientation.k -
            2*orientation.r*orientation.j;
    transformMatrix.data[9] = 2*orientation.j*orientation.k +
            2*orientation.r*orientation.i;
    transformMatrix.data[10] = 1-2*orientation.i*orientation.i-
            2*orientation.j*orientation.j;
    transformMatrix.data[11] = position.z;
}


static inline void _transformInertiaTensor(Matrix3 &iitWorld,
                                           const Quaternion &q,
                                           const Matrix3 &iitBody,
                                           const Matrix4 &rotmat)
{
    real t4 = rotmat.data[0]*iitBody.data[0]+
        rotmat.data[1]*iitBody.data[3]+
        rotmat.data[2]*iitBody.data[6];
    real t9 = rotmat.data[0]*iitBody.data[1]+
        rotmat.data[1]*iitBody.data[4]+
        rotmat.data[2]*iitBody.data[7];
    real t14 = rotmat.data[0]*iitBody.data[2]+
        rotmat.data[1]*iitBody.data[5]+
        rotmat.data[2]*iitBody.data[8];
    real t28 = rotmat.data[4]*iitBody.data[0]+
        rotmat.data[5]*iitBody.data[3]+
        rotmat.data[6]*iitBody.data[6];
    real t33 = rotmat.data[4]*iitBody.data[1]+
        rotmat.data[5]*iitBody.data[4]+
        rotmat.data[6]*iitBody.data[7];
    real t38 = rotmat.data[4]*iitBody.data[2]+
        rotmat.data[5]*iitBody.data[5]+
        rotmat.data[6]*iitBody.data[8];
    real t52 = rotmat.data[8]*iitBody.data[0]+
        rotmat.data[9]*iitBody.data[3]+
        rotmat.data[10]*iitBody.data[6];
    real t57 = rotmat.data[8]*iitBody.data[1]+
        rotmat.data[9]*iitBody.data[4]+
        rotmat.data[10]*iitBody.data[7];
    real t62 = rotmat.data[8]*iitBody.data[2]+
        rotmat.data[9]*iitBody.data[5]+
        rotmat.data[10]*iitBody.data[8];

    iitWorld.data[0] = t4*rotmat.data[0]+
        t9*rotmat.data[1]+
        t14*rotmat.data[2];
    iitWorld.data[1] = t4*rotmat.data[4]+
        t9*rotmat.data[5]+
        t14*rotmat.data[6];
    iitWorld.data[2] = t4*rotmat.data[8]+
        t9*rotmat.data[9]+
        t14*rotmat.data[10];
    iitWorld.data[3] = t28*rotmat.data[0]+
        t33*rotmat.data[1]+
        t38*rotmat.data[2];
    iitWorld.data[4] = t28*rotmat.data[4]+
        t33*rotmat.data[5]+
        t38*rotmat.data[6];
    iitWorld.data[5] = t28*rotmat.data[8]+
        t33*rotmat.data[9]+
        t38*rotmat.data[10];
    iitWorld.data[6] = t52*rotmat.data[0]+
        t57*rotmat.data[1]+
        t62*rotmat.data[2];
    iitWorld.data[7] = t52*rotmat.data[4]+
        t57*rotmat.data[5]+
        t62*rotmat.data[6];
    iitWorld.data[8] = t52*rotmat.data[8]+
        t57*rotmat.data[9]+
        t62*rotmat.data[10];
}


void RigidBody::calculateDerivedData()
{
    // calculate transrom matrix based on position and orientation
    _calculateTransformMatrix(transformMatrix, position, orientation);

    // calculate inverse inertia tensor in world space
    _transformInertiaTensor(inverseInertiaTensorWorld,
        orientation,
        inverseInertiaTensor,
        transformMatrix);
}

void RigidBody::setInertiaTensor(const Matrix3 &inertiaTensor)
{
    inverseInertiaTensor.setInverse(inertiaTensor);
    // TODO: Perform a validity check in an assert.

}

Vector3 RigidBody::getPointInWorldSpace(const Vector3 &point) const
{
    return transformMatrix.transform(point);
}

void RigidBody::addForceAtBodyPoint(const Vector3 &force, const Vector3 &point)
{
    Vector3 pt = getPointInWorldSpace(point);
    addForceAtPoint(pt, force);
}

bool RigidBody::hasFiniteMass() const
{
    return inverseMass >= 0.0f;
}

real RigidBody::getMass() const
{
    if (inverseMass == 0) {
        return 10000.0f; // FIX to have REAL_MAX
    } else {
        return ((real)1.0)/inverseMass;
    }
}

void RigidBody::addForceAtPoint(const Vector3 &force, const Vector3 &point)
{
    Vector3 pt = point;
    pt -= position;

    forceAccum += force;
    torqueAccum += pt % force;

    isAwake = true;
}

void RigidBody::addForce(const Vector3 &force)
{
    forceAccum +=force;
}

void RigidBody::clearAccumulators()
{
    forceAccum.clear();
    torqueAccum.clear();
}

void RigidBody::integrate(real duration)
{
    clearAccumulators();
}

}
