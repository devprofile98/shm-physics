#ifndef CYCLON_FORCE_GENERATOR
#define CYCLON_FORCE_GENERATOR

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

    // force generator for spring like object
    // -------------------------------------
    class ParticleSpring: ParticleForceGenerator{
            Particle *other; // particle at the other end
            real springConstant;
            real restLength;

        public:
            ParticleSpring(Particle *other, real springConstant, real restLength);

            virtual void updateForce(Particle *particle, real duration) override;

    };

    // force generator for anchor spring like object
    // ---------------------------------------------
    class ParticleAnchorSpring: ParticleForceGenerator{
        Vector3 *anchor;
        real springConstant;
        real restLength;

    public:
        ParticleAnchorSpring(Vector3 *anchor, real springConstant, real restLength);
        virtual void updateForce(Particle *particle, real duration) override;
    };

    // force generator for bungee like spring object
    // ---------------------------------------------
    class ParticleBungee:ParticleForceGenerator{
        Particle *other; // particle at the other end
        real springConstant;
        real restLength;

    public:
        ParticleBungee(Particle *other, real springConstant, real restLength);

        virtual void updateForce(Particle *particle, real duration) override;
    };

    // force generator for buoyancy
    // ----------------------------
    class ParticleBuoyancy: ParticleForceGenerator{
        real maxDepth;
        real volume;
        real waterHeight;
        real liquidDensity;
    public:
        ParticleBuoyancy(real maxDepth, real volume, real waterHeight, real liquidDensity=1000.0f);
        virtual void updateForce(Particle *particle, real duration) override;
    };

}
#endif //CYCLON_FORCE_GENERATOR
