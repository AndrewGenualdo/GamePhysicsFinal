//
// Created by andrew.genualdo on 9/11/2025.
//

#include "springForceGenerator.h"

namespace cyclone {
    SpringForceGenerator::SpringForceGenerator() {
        this->anchorPosition = Vector3(0, 0, 0);
        this->restLength = 1.0f;
        this->springConstant = 1.0f;
        this->damping = 1.0f;
        this->isBungee = false;
    }

    SpringForceGenerator::SpringForceGenerator(Vector3 anchorPosition, real restLength, real springConstant, real damping, bool isBungee) {
        this->anchorPosition = anchorPosition;
        this->restLength = restLength;
        this->springConstant = springConstant;
        this->damping = damping;
        this->isBungee = isBungee;
    }

    Vector3 SpringForceGenerator::getAnchorPosition() const {
        return anchorPosition;
    }

    void SpringForceGenerator::setAnchorPosition(const Vector3 anchorPosition) {
        this->anchorPosition = anchorPosition;
    }

    real SpringForceGenerator::getRestLength() const {
        return restLength;
    }

    void SpringForceGenerator::setRestLength(const real restLength) {
        this->restLength = restLength;
    }

    real SpringForceGenerator::getSpringConstant() const {
        return springConstant;
    }

    void SpringForceGenerator::setSpringConstant(const real springConstant) {
        this->springConstant = springConstant;
    }

    real SpringForceGenerator::getDamping() const {
        return damping;
    }

    void SpringForceGenerator::setDamping(const real damping) {
        this->damping = damping;
    }

    bool SpringForceGenerator::getIsBungee() const {
        return isBungee;
    }

    void SpringForceGenerator::setIsBungee(const bool isBungee) {
        this->isBungee = isBungee;
    }

    void SpringForceGenerator::updateForce(Particle *particle, real time) {
        const Vector3 dist = anchorPosition - particle->getPosition();
        if (isBungee && dist.magnitude() < restLength) return;
        particle->addForce(dist * springConstant - particle->getVelocity() * damping);
    }
} // cyclone