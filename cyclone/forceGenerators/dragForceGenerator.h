//
// Created by andrew.genualdo on 9/11/2025.
//

#ifndef DRAGFORCEGENERATOR_H
#define DRAGFORCEGENERATOR_H
#include "particleForceGenerator.h"

namespace cyclone {

class DragForceGenerator : public ParticleForceGenerator {

    real linearCoefficient;
    real exponentialCoefficient;

public:
    DragForceGenerator();
    DragForceGenerator(real linearCoefficient, real exponentialCoefficient);

    [[nodiscard]] real getLinearCoefficient() const;
    void setLinearCoefficient(real linearCoefficient);

    [[nodiscard]] real getExponentialCoefficient() const;
    void setExponentialCoefficient(real exponentialCoefficient);

    void updateForce(Particle* particle, real time) override;

};

} // cyclone

#endif //DRAGFORCEGENERATOR_H
