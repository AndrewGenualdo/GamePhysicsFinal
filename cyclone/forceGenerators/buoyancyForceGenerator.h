//
// Created by andrew.genualdo on 9/11/2025.
//

#ifndef BUOYANCYFORCEGENERATOR_H
#define BUOYANCYFORCEGENERATOR_H
#include "particleForceGenerator.h"

namespace cyclone {

class BuoyancyForceGenerator : public ParticleForceGenerator {

    real liquidHeight; //y coordinate of the water plane
    real liquidDensity;
    real objectVolume;
    real objectMaxDepth;
    real gravity;

public:

    BuoyancyForceGenerator();
    BuoyancyForceGenerator(real waterHeight, real liquidDensity, real objectVolume, real objectMaxDepth, real gravity);

    [[nodiscard]] real getLiquidHeight() const;
    void setLiquidHeight(real waterHeight);

    [[nodiscard]] real getLiquidDensity() const;
    void setLiquidDensity(real liquidDensity);

    [[nodiscard]] real getObjectVolume() const;
    void setObjectVolume(real objectVolume);

    [[nodiscard]] real getObjectMaxDepth() const;
    void setObjectMaxDepth(real objectMaxDepth);

    [[nodiscard]] real getGravity() const;
    void setGravity(real gravity);

    void updateForce(Particle *particle, real duration) override;

};

} // cyclone

#endif //BUOYANCYFORCEGENERATOR_H
