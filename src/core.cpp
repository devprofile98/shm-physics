#include "core.hpp"


namespace cyclon {

real Matrix4::getDeterminant() const
{
    return data[8]*data[5]*data[2] +
            data[4]*data[9]*data[2] +
            data[8]*data[1]*data[6] -
            data[0]*data[9]*data[6] -
            data[4]*data[1]*data[10]+
            data[0]*data[5]*data[10];
}

void Matrix4::setInverse(const Matrix4 &m)
{
    // Make sure the determinant is non-zero.
    real det = getDeterminant();
    if (det == 0) return;
    det = ((real)1.0)/det;

    data[0] = (-m.data[9]*m.data[6]+m.data[5]*m.data[10])*det;
    data[4] = (m.data[8]*m.data[6]-m.data[4]*m.data[10])*det;
    data[8] = (-m.data[8]*m.data[5]+m.data[4]*m.data[9])*det;

    data[1] = (m.data[9]*m.data[2]-m.data[1]*m.data[10])*det;
    data[5] = (-m.data[8]*m.data[2]+m.data[0]*m.data[10])*det;
    data[9] = (m.data[8]*m.data[1]-m.data[0]*m.data[9])*det;

    data[2] = (-m.data[5]*m.data[2]+m.data[1]*m.data[6])*det;
    data[6] = (+m.data[4]*m.data[2]-m.data[0]*m.data[6])*det;
    data[10] = (-m.data[4]*m.data[1]+m.data[0]*m.data[5])*det;

    data[3] = (m.data[9]*m.data[6]*m.data[3]
            -m.data[5]*m.data[10]*m.data[3]
            -m.data[9]*m.data[2]*m.data[7]
            +m.data[1]*m.data[10]*m.data[7]
            +m.data[5]*m.data[2]*m.data[11]
            -m.data[1]*m.data[6]*m.data[11])*det;
    data[7] = (-m.data[8]*m.data[6]*m.data[3]
            +m.data[4]*m.data[10]*m.data[3]
            +m.data[8]*m.data[2]*m.data[7]
            -m.data[0]*m.data[10]*m.data[7]
            -m.data[4]*m.data[2]*m.data[11]
            +m.data[0]*m.data[6]*m.data[11])*det;
    data[11] =(m.data[8]*m.data[5]*m.data[3]
            -m.data[4]*m.data[9]*m.data[3]
            -m.data[8]*m.data[1]*m.data[7]
            +m.data[0]*m.data[9]*m.data[7]
            +m.data[4]*m.data[1]*m.data[11]
            -m.data[0]*m.data[5]*m.data[11])*det;
}

Matrix4 Matrix4::inverse() const
{
    Matrix4 result;
    result.setInverse(*this);
    return result;
}

void Matrix4::invert()
{
    setInverse(*this);
}

void Matrix4::setOrientationAndPos(const Quaternion &q, const Vector3 &pos)
{
    {
        data[0] = 1 - (2*q.j*q.j + 2*q.k*q.k);
        data[1] = 2*q.i*q.j + 2*q.k*q.r;
        data[2] = 2*q.i*q.k - 2*q.j*q.r;
        data[3] = pos.x;

        data[4] = 2*q.i*q.j - 2*q.k*q.r;
        data[5] = 1 - (2*q.i*q.i  + 2*q.k*q.k);
        data[6] = 2*q.j*q.k + 2*q.i*q.r;
        data[7] = pos.y;

        data[8] = 2*q.i*q.k + 2*q.j*q.r;
        data[9] = 2*q.j*q.k - 2*q.i*q.r;
        data[10] = 1 - (2*q.i*q.i  + 2*q.j*q.j);
        data[11] = pos.z;
    }
}

// param:vector can be pass by value, because in function we are
// copieng it
Vector3 Matrix4::transformInverse(const Vector3 &vector) const
{
    Vector3 temp = vector;
    temp.x -= data[3];
    temp.y -= data[7];
    temp.z -= data[11];

    return Vector3{
        temp.x * data[0] +
        temp.y * data[4] +
        temp.z * data[8],

        temp.x * data[1] +
        temp.y * data[5] +
        temp.z * data[9],

        temp.x * data[2] +
        temp.y * data[6] +
        temp.z * data[10]
    };
}

Vector3 Matrix4::transformInverseDirection(const Vector3 &vector) const
{
    return Vector3{
        vector.x * data[0] +
        vector.y * data[4] +
        vector.z * data[8],

        vector.x * data[1] +
        vector.y * data[5] +
        vector.z * data[9],

        vector.x * data[2] +
        vector.y * data[6] +
        vector.z * data[10]
    };
}

Vector3 Matrix4::transformDirection(const Vector3 &vector) const
{
    return Vector3{
        vector.x * data[0] +
        vector.y * data[1] +
        vector.z * data[2],

        vector.x * data[4] +
        vector.y * data[5] +
        vector.z * data[6],

        vector.x * data[8] +
        vector.y * data[9] +
        vector.z * data[10]
    };
}

void Matrix3::setTranspose(const Matrix3 &m)
{
    data[0] = m.data[0];
    data[1] = m.data[3];
    data[2] = m.data[6];
    data[3] = m.data[1];
    data[4] = m.data[4];
    data[5] = m.data[7];
    data[6] = m.data[2];
    data[7] = m.data[5];
    data[8] = m.data[8];
}

Matrix3 Matrix3::transpose() const
{
    Matrix3 result;
    result.setTranspose(*this);
    return result;
}

void Matrix3::setOrientation(const Quaternion &q)
{
    data[0] = 1 - (2*q.j*q.j + 2*q.k*q.k);
    data[1] = 2*q.i*q.j + 2*q.k*q.r;
    data[2] = 2*q.i*q.k - 2*q.j*q.r;
    data[3] = 2*q.i*q.j - 2*q.k*q.r;
    data[4] = 1 - (2*q.i*q.i  + 2*q.k*q.k);
    data[5] = 2*q.j*q.k + 2*q.i*q.r;
    data[6] = 2*q.i*q.k + 2*q.j*q.r;
    data[7] = 2*q.j*q.k - 2*q.i*q.r;
    data[8] = 1 - (2*q.i*q.i  + 2*q.j*q.j);
}

Quaternion::Quaternion(real r, real i, real j, real k)
    : r(r), i(i), j(j), k(k) {}

void Quaternion::normalize()
{
    real d = r*r + i*i + j*j + k+k;
    if (d == 0){
        r = 1;
        return;
    }

    d = ((real)1.0)/real_sqrt(d);
    r *= d; i *= d; j *= d; k *= d;

}

void Quaternion::rotateByVector(const Vector3 &vector)
{
    Quaternion q{0, vector.x, vector.y, vector.z};
    (*this) *= q;
}

void Quaternion::addScaledVector(const Vector3 &vector, real scale)
{
    Quaternion q{
        0,
        vector.x * scale,
                vector.y * scale,
                vector.z * scale,
    };

    q *= *this;
    r += q.r * ((real)0.5);
    i += q.i * ((real)0.5);
    j += q.j * ((real)0.5);
    k += q.k * ((real)0.5);
}

void Quaternion::operator *= (const Quaternion &multiplier)
{
    Quaternion q = *this;
    r = q.r*multiplier.r - q.i*multiplier.i -
            q.j*multiplier.j - q.k*multiplier.k;
    i = q.r*multiplier.i + q.i*multiplier.r +
            q.j*multiplier.k - q.k*multiplier.j;
    j = q.r*multiplier.j + q.j*multiplier.r +
            q.k*multiplier.i - q.i*multiplier.k;
    k = q.r*multiplier.k + q.k*multiplier.r +
            q.i*multiplier.j - q.j*multiplier.i;
}

}
