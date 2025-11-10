//
// Created by andrew.genualdo on 9/11/2025.
//

#include "dragForceGenerator.h"

namespace cyclone {
    DragForceGenerator::DragForceGenerator() {
        this->linearCoefficient = 1;
        this->exponentialCoefficient = 1;
    }

    DragForceGenerator::DragForceGenerator(const real linearCoefficient, const real exponentialCoefficient) {
        this->linearCoefficient = linearCoefficient;
        this->exponentialCoefficient = exponentialCoefficient;
    }

    real DragForceGenerator::getLinearCoefficient() const {
        return linearCoefficient;
    }

    void DragForceGenerator::setLinearCoefficient(real linearCoefficient) {
        this->linearCoefficient = linearCoefficient;
    }

    real DragForceGenerator::getExponentialCoefficient() const {
        return exponentialCoefficient;
    }

    void DragForceGenerator::setExponentialCoefficient(real exponentialCoefficient) {
        this->exponentialCoefficient = exponentialCoefficient;
    }

    void DragForceGenerator::updateForce(Particle *particle, real time) {

        Vector3 force = particle->getVelocity();

        real dragCoeff = force.magnitude();
        dragCoeff = linearCoefficient * dragCoeff + exponentialCoefficient * dragCoeff * dragCoeff;

        force.normalise();
        force *= -dragCoeff;
        particle->addForce(force);


    }
} // cyclone