#ifndef CYCLON_CORE
#define CYCLON_CORE

#include <math.h>
#include <cstdint>
#include "precision.hpp"

namespace cyclon {

class Vector3{
public:
    Vector3(): x(0), y(0), z(0){}
    Vector3(const real x, const real y, const real z):
        x(x), y(y), z(z){}

    void invert(){
        x *= -1;
        y *= -1;
        z *= -1;
    }
    real x, y, z;

    real magnitude() const{
        return real_sqrt(float(x*x + y*y + z*z));
    }

    real squareMagnitude(){
        return x*x + y*y + z*z;
    }

    void normalize(){
        real l = magnitude();
        if (l>0){
            (*this) *= ((real)1)/l;
        }
    }

    // ** member function
    // ------------------------

    void addScaledVector(const Vector3& vector, real scale){
        // this + vector*sclae
        x += vector.x * scale;
        y += vector.y * scale;
        z += vector.z * scale;
    }

    // component product
    Vector3 componentProduct(const Vector3& vector) const {
        return Vector3(x*vector.x, y*vector.y, z*vector.z);
    }

    void componentProduct(const Vector3& vector){
        x *= vector.x;
        y *= vector.y;
        z *= vector.z;
    }

    // explicit function for dot product
    real scalarProduct(const Vector3& vector){
        return x*vector.x + y*vector.y + z*vector.z;
    }

    // function for Cross product of another vector with this
    Vector3 vectorProduct(const Vector3& vector) const {
        return Vector3(
                    y*vector.z - z*vector.y,
                    z*vector.x - x*vector.z,
                    x*vector.y - y*vector.x
                    );
    }

    // **  over load the operators
    // ------------------------
    void operator *=(const real value){
        x *= value;
        y *= value;
        z *= value;
    }

    // vector by vector multiplication
    Vector3 operator *(Vector3 vector) const {
        return Vector3(x * vector.x, y * vector.y, z * vector.z);
    }

    real operator *(const Vector3 &v) const {
        return x*v.x + y*v.y + z*v.z;
    }

    Vector3 operator *(const real value) const{
        return Vector3{x*value, y*value, z*value};
    }

    // adding other vector to this
    void operator +=(const Vector3& vector){
        x += vector.x;
        y += vector.y;
        z += vector.z;
    }
    // add other vector and return the result
    Vector3 operator +(const Vector3& vector) const {
        return Vector3(x+vector.x, y+vector.y, z+vector.z);
    }

    // subtract other vector
    void operator -=(const Vector3& vector){
        x -= vector.x;
        y -= vector.y;
        z -= vector.z;
    }

    // subtract other vector and return
    Vector3 operator -(const Vector3& vector) const {
        return Vector3(x-vector.x, y-vector.y, z-vector.z);
    }

    // overload % operator for Cross product
    Vector3 operator %(const Vector3& vector) const {
        return Vector3(
                    y*vector.z - z*vector.y,
                    z*vector.x - x*vector.z,
                    x*vector.y - y*vector.x
                    );
    }

    // update 'this' with new vector
    void operator %=(const Vector3& vector){
        *this = vectorProduct(vector);
    }

    void clear(){
        x = y = z = 0;
    }

private:
    real pad; // ensure 4-word alignment
};


class Quaternion{
public:

    Quaternion(real r=0,real i=0,real j=0,real k=0);

    union{
        struct{
            // holds data in anonymous struct and members
            real r, i, j, k;
        };

        real data[4];   // holds data in array
    };

    // we need to nomalize quaternion, because in long run
    // flating points component will change and the magnitude
    // of quaternion would not be equal to 1, and is wrong
    void normalize();
    void rotateByVector(const Vector3& vector);
    void addScaledVector(const Vector3& vector, real scale);

    void operator *= (const Quaternion& multiplier);

};

// represent a 3 x 3 matrix
// ------------------------------
class Matrix3{
public:

    Matrix3()
    {
        data[0] = data[1] = data[2] = data[3] = data[4] = data[5] =
            data[6] = data[7] = data[8] = 0;
    }

    Matrix3(real c0, real c1, real c2, real c3, real c4, real c5,
        real c6, real c7, real c8)
    {
        data[0] = c0; data[1] = c1; data[2] = c2;
        data[3] = c3; data[4] = c4; data[5] = c5;
        data[6] = c6; data[7] = c7; data[8] = c8;
    }

    real data[9];

    Matrix3 operator * (const Matrix3& o) const{
        return Matrix3{
            data[0] * o.data[0] + data[1] * o.data[3] + data[2] * o.data[6],
                    data[0] * o.data[1] + data[1] * o.data[4] + data[2] * o.data[7],
                    data[0] * o.data[2] + data[1] * o.data[5] + data[2] * o.data[8],

                    data[3] * o.data[0] + data[4] * o.data[3] + data[5] * o.data[6],
                    data[3] * o.data[1] + data[4] * o.data[4] + data[5] * o.data[7],
                    data[3] * o.data[2] + data[4] * o.data[5] + data[5] * o.data[8],

                    data[6] * o.data[0] + data[7] * o.data[3] + data[8] * o.data[6],
                    data[6] * o.data[1] + data[7] * o.data[4] + data[8] * o.data[7],
                    data[6] * o.data[2] + data[7] * o.data[5] + data[8] * o.data[8],
        };
    }

    void setInverse(const Matrix3& m){
        real t4 = m.data[0] * m.data[4];
        real t6 = m.data[0] * m.data[5];
        real t8 = m.data[1] * m.data[3];
        real t10 = m.data[2] * m.data[3];
        real t12 = m.data[1] * m.data[6];
        real t14 = m.data[2] * m.data[6];

        // the determinant
        real t16 = (t4*m.data[8] - t6*m.data[7] - t8*m.data[8] + t10*m.data[7] + t12*m.data[5] + t14*m.data[4]);

        if (t16 == (real)0.0f) return;
        real t17 = 1/t16;
        data[0] = (m.data[4]*m.data[8] - m.data[5]*m.data[7])*t17;
        data[1] = -(m.data[1]*m.data[8] - m.data[2]*m.data[7])*t17;
        data[2] = (m.data[1]*m.data[5] - m.data[2]*m.data[4])*t17;
        data[3] = -(m.data[3]*m.data[8] - m.data[5]*m.data[6])*t17;
        data[4] = (m.data[0]*m.data[8] - t14)*t17;
        data[5] = -(t6 - t10)*t17;
        data[6] = (m.data[3]*m.data[7] - m.data[4]*m.data[6])*t17;
        data[7] = -(m.data[0]*m.data[7] - t12)*t17;
        data[8] = (t4 - t8)*t17;
    }

    Matrix3 inverse() {
        Matrix3 result;
        result.setInverse(*this);
        return result;
    }

    void invert(){
        setInverse(*this);
    }

    void setTranspose(const Matrix3& m);
    Matrix3 transpose() const;
    void setOrientation(const Quaternion &q);
};


// represent a 3 x 4 matrix
// ------------------------------
class Matrix4{
public:
    real data[12];

    Vector3 operator *(const Vector3& vector) const{
        return Vector3{
            data[0] * vector.x + data[1] * vector.y + data[2] * vector.z + data[3],
                    data[4] * vector.x + data[5] * vector.y + data[6] * vector.z + data[7],
                    data[8] * vector.x + data[9] * vector.y + data[10] * vector.z + data[11],
        };
    }

    Matrix4 operator * (const Matrix4& o) const{
        Matrix4 result;
        result.data[0] = data[0] * o.data[0]+
                data[1] * o.data[4] + data[2] * o.data[8];
        result.data[1] = data[0] * o.data[1]+
                data[1] * o.data[5] + data[2] * o.data[9];
        result.data[2] = data[0] * o.data[2]+
                data[1] * o.data[6] + data[2] * o.data[10];
        result.data[3] = data[0] * o.data[3]+
                data[1] * o.data[7] + data[2] * o.data[11] + data[3];

        result.data[4] = data[4] * o.data[0]+
                data[5] * o.data[4] + data[6] * o.data[8];
        result.data[5] = data[4] * o.data[1]+
                data[5] * o.data[5] + data[6] * o.data[9];
        result.data[6] = data[4] * o.data[2]+
                data[5] * o.data[6] + data[6] * o.data[10];
        result.data[7] = data[4] * o.data[3]+
                data[5] * o.data[7] + data[6] * o.data[11] + data[7];

        result.data[8] = data[8] * o.data[0]+
                data[9] * o.data[4] + data[10] * o.data[8];
        result.data[9] = data[8] * o.data[1]+
                data[9] * o.data[5] + data[10] * o.data[9];
        result.data[10] = data[8] * o.data[2]+
                data[9] * o.data[6] + data[10] * o.data[10];
        result.data[11] = data[8] * o.data[3]+
                data[9] * o.data[7] + data[10] * o.data[11] + data[11];

        return result;
    }

    real getDeterminant() const;
    void setInverse(const Matrix4& m);
    Matrix4 inverse() const;
    void invert();
    void setOrientationAndPos(const Quaternion &q, const Vector3 &pos);

    Vector3 transformInverse(const Vector3 &vector) const;
    Vector3 transformInverseDirection(const Vector3& vector) const;
    Vector3 transformDirection(const Vector3& vector) const;
    Vector3 transform(const Vector3 &vector) const
    {
        return (*this) * vector;
    }

};

}
#endif //CYCLON_CORE
