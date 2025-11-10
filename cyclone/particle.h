//
// Created by andrew.genualdo on 9/1/2025.
//

#ifndef PARTICLE_H
#define PARTICLE_H
#include "core.h"


namespace cyclone {
    class Particle {
        Vector3 position;
        Vector3 velocity;
        Vector3 acceleration;
        real damping;
        real mass;
        real inverseMass;
        Vector3 forceAccum;

    public:

        Particle();
        Particle(Vector3 position, Vector3 velocity, Vector3 acceleration, real damping, real mass);

        void setMass(real mass);
        void addForce(Vector3 force);
        void clearAccumulator();
        void integrate(real deltaTime);

        [[nodiscard]] Vector3 getPosition() const { return position; }
        [[nodiscard]] Vector3 getVelocity() const { return velocity; }
        [[nodiscard]] Vector3 getAcceleration() const { return acceleration; }
        [[nodiscard]] real getDamping() const { return damping; }

        void setPosition(const Vector3 newPosition) { position = newPosition; }
        void setVelocity(const Vector3 newVelocity) { velocity = newVelocity; }
        void setAcceleration(const Vector3 newVelocity) { acceleration = newVelocity; }
        void setDamping(const real newDamping) { damping = newDamping; }

        Particle& operator = (const Particle &rhs);
    };
}




#endif //PARTICLE_H
