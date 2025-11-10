//
// Created by andrew.genualdo on 9/8/2025.
//

#ifndef PARTICLEFORCEGENERATOR_H
#define PARTICLEFORCEGENERATOR_H
#include "../particle.h"

namespace cyclone {

    class ParticleForceGenerator {
    public:
        ParticleForceGenerator() = default;
        virtual ~ParticleForceGenerator() = default;

    private:
        virtual void updateForce(Particle *particle, real duration) = 0;



    };

    class ParticleForceRegistry {
        void add(Particle *particle, ParticleForceGenerator *fg);
        void updateForces(real duration);
    };

} // cyclone




#endif //PARTICLEFORCEGENERATOR_H
