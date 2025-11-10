//
// Created by andrew.genualdo on 9/11/2025.
//

#include "buoyancyForceGenerator.h"

#include <algorithm>

namespace cyclone {
    BuoyancyForceGenerator::BuoyancyForceGenerator() {
        this->liquidHeight = 0;
        this->liquidDensity = 0;
        this->objectMaxDepth = 1;
        this->objectVolume = 4.0F / 3.0F * M_PI * pow(objectMaxDepth, 3);
        this->gravity = -2.5F;
    }

    BuoyancyForceGenerator::BuoyancyForceGenerator(const real waterHeight, const real liquidDensity, const real objectVolume, const real objectMaxDepth, const real gravity) {
        this->liquidHeight = waterHeight;
        this->liquidDensity = liquidDensity;
        this->objectVolume = objectVolume;
        this->objectMaxDepth = objectMaxDepth;
        this->gravity = -gravity;
    }

    real BuoyancyForceGenerator::getLiquidHeight() const {
        return liquidHeight;
    }

    void BuoyancyForceGenerator::setLiquidHeight(const real waterHeight) {
        this->liquidHeight = waterHeight;
    }

    real BuoyancyForceGenerator::getLiquidDensity() const {
        return liquidDensity;
    }

    void BuoyancyForceGenerator::setLiquidDensity(const real liquidDensity) {
        this->liquidDensity = liquidDensity;
    }

    real BuoyancyForceGenerator::getObjectVolume() const {
        return objectVolume;
    }

    void BuoyancyForceGenerator::setObjectVolume(const real objectVolume) {
        this->objectVolume = objectVolume;
        this->objectMaxDepth = pow((3.0F * objectVolume) / (4.0F * M_PI), 1.0F / 3.0F);
    }

    real BuoyancyForceGenerator::getObjectMaxDepth() const {
        return objectMaxDepth;
    }

    void BuoyancyForceGenerator::setObjectMaxDepth(const real objectMaxDepth) {
        this->objectMaxDepth = objectMaxDepth;
        this->objectVolume = 4.0F / 3.0F * M_PI * pow(objectMaxDepth, 3);
    }

    real BuoyancyForceGenerator::getGravity() const {
        return gravity;
    }

    void BuoyancyForceGenerator::setGravity(const real gravity) {
        this->gravity = -gravity;
    }

    void BuoyancyForceGenerator::updateForce(Particle *particle, real duration) {
        real d = (particle->getPosition().y - liquidHeight - objectMaxDepth) / (2.0F * objectMaxDepth);
        d = static_cast<real>(std::clamp(static_cast<double>(1.0F - d), 0.0, 1.0));
        particle->addForce(Vector3::UP * objectVolume * -liquidDensity * d * gravity);
    }
} // cyclone