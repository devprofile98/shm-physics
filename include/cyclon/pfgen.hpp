#ifndef CYCLON_FORCE_GENERATOR
#define CYCLON_FORCEGENERATOR

#include "particle.hpp"
#include <vector>

namespace cyclon {

    class ParticleForceGenerator{
    public:
        virtual void updateForce(Particle *particle, real duration) = 0;
    };

    class ParticleForceRegistry{
    protected:
        struct ParticleForceRegistration{
            Particle *particle;
            ParticleForceGenerator *fg;
        };

//        using Registry = std::vector<ParticleForceRegistration>;
        typedef std::vector<ParticleForceRegistration> Registry;
        Registry registration;

    public:
        void add(Particle *particle,ParticleForceGenerator *fg);
        void remove(Particle *particle,ParticleForceGenerator *fg);
        void clear();
        void updateForces(real duration);
    };

    class ParticleGravity: public ParticleForceGenerator{
    public:
        ParticleGravity(const Vector3 &gravity);
        virtual void updateForce(Particle *particle, real duration) override;
    private:
        Vector3 gravity;
    };

}
#endif //CYCLON_FORCEGENERATOR
