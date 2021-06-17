#ifndef CYCLON_CORE
#define CYCLON_CORE

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

    private:
        real pad; // ensure 4-word alignment
    };
}

#endif //CYCLON_CORE
