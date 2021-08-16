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
}

class Matrix3{
public:
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
};


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




};

#endif //CYCLON_CORE
