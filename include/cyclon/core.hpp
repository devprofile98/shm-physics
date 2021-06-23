#ifndef CYCLON_CORE
#define CYCLON_CORE

#include <math.h>
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
        Vector3 operator *(const Vector3 vector) const {
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

#endif //CYCLON_CORE
