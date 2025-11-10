//
// Created by andrew.genualdo on 9/11/2025.
//

#ifndef SPRINGFORCEGENERATOR_H
#define SPRINGFORCEGENERATOR_H
#include "particleForceGenerator.h"

namespace cyclone {

class SpringForceGenerator : public ParticleForceGenerator {

    Vector3 anchorPosition;
    real restLength;
    real springConstant;
    real damping; //-cv described in lecture, not book
    bool isBungee; //if true, no force to push it away from anchorPosition will be applied

public:

    SpringForceGenerator();
    SpringForceGenerator(Vector3 anchorPosition, real restLength, real springConstant, real damping, bool isBungee);

    [[nodiscard]] Vector3 getAnchorPosition() const;
    void setAnchorPosition(Vector3 anchorPosition);

    [[nodiscard]] real getRestLength() const;
    void setRestLength(real restLength);

    [[nodiscard]] real getSpringConstant() const;
    void setSpringConstant(real springConstant);

    [[nodiscard]] real getDamping() const;
    void setDamping(real damping);

    [[nodiscard]] bool getIsBungee() const;
    void setIsBungee(bool isBungee);

    void updateForce(Particle* particle, real time) override;

};

} // cyclone

#endif //SPRINGFORCEGENERATOR_H
