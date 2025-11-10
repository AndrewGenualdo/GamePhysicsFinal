//
// Created by andrew.genualdo on 9/1/2025.
//

#include "particle.h"

cyclone::Particle::Particle()  {
    position = Vector3(0, 0, 0);
    velocity = Vector3(0, 0, 0);
    acceleration = Vector3(0, 0, 0);
    damping = 1;
    mass = 1;
    inverseMass = 1;
    forceAccum = Vector3(0, 0, 0);
}

cyclone::Particle::Particle(const Vector3 position, const Vector3 velocity, const Vector3 acceleration, const real damping, const real mass) {
    this->position = position;
    this->velocity = velocity;
    this->acceleration = acceleration;
    this->damping = damping;
    this->mass = mass;
    this->inverseMass = static_cast<real>(1.0) / mass;
}

void cyclone::Particle::setMass(const real mass) {
    this->mass = mass;
    this->inverseMass = static_cast<real>(1.0) / mass;
}

void cyclone::Particle::addForce(const Vector3 force) {
    forceAccum += force;
}

void cyclone::Particle::clearAccumulator() {
    forceAccum = Vector3(0, 0, 0);
}

void cyclone::Particle::integrate(real deltaTime) {
    this->position += velocity * deltaTime;
    Vector3 finalAccel = acceleration;
    finalAccel += forceAccum * inverseMass;
    this->velocity += finalAccel * deltaTime;
    this->velocity *= pow(damping, deltaTime);
    clearAccumulator();
}

cyclone::Particle & cyclone::Particle::operator=(const Particle &rhs) {
    this->acceleration = rhs.acceleration;
    this->velocity = rhs.velocity;
    this->position = rhs.position;
    this->damping = rhs.damping;
    this->mass = rhs.mass;
    this->inverseMass = rhs.inverseMass;
    this->forceAccum = rhs.forceAccum;
    return *this;
}