//
// Created by andrew.genualdo on 9/18/2025.
//

#include "rigidbody.h"

cyclone::Vector3* cyclone::Rigidbody::getPosition() {
    return &position;
}

void cyclone::Rigidbody::setPosition(const Vector3 &position) {
    this->position = position;
}

cyclone::Vector3* cyclone::Rigidbody::getVelocity() {
    return &velocity;
}

void cyclone::Rigidbody::setVelocity(const Vector3 &velocity) {
    this->velocity = velocity;
}

cyclone::Vector3* cyclone::Rigidbody::getAcceleration() {
    return &acceleration;
}

void cyclone::Rigidbody::setAcceleration(const Vector3 &acceleration) {
    this->acceleration = acceleration;
}

cyclone::real* cyclone::Rigidbody::getLinearDamping() {
    return &linearDamping;
}

void cyclone::Rigidbody::setLinearDamping(const real linearDamping) {
    this->linearDamping = linearDamping;
}

cyclone::real* cyclone::Rigidbody::getInverseMass() {
    return &inverseMass;
}

void cyclone::Rigidbody::setInverseMass(const real inverseMass) {
    this->inverseMass = inverseMass;
}

cyclone::Quaternion* cyclone::Rigidbody::getOrientation() {
    return &orientation;
}

void cyclone::Rigidbody::setOrientation(const Quaternion &orientation) {
    this->orientation = orientation;
}

cyclone::Vector3* cyclone::Rigidbody::getAngularVelocity() {
    return &angularVelocity;
}

void cyclone::Rigidbody::setAngularVelocity(const Vector3 &angularVelocity) {
    this->angularVelocity = angularVelocity;
}

cyclone::Vector3* cyclone::Rigidbody::getTorqueAccum() {
    return &torqueAccum;
}

void cyclone::Rigidbody::setTorqueAccum(const Vector3 &torqueAccum) {
    this->torqueAccum = torqueAccum;
}

cyclone::Matrix3* cyclone::Rigidbody::getInverseInertiaTensor() {
    return &inverseInertiaTensor;
}

void cyclone::Rigidbody::setInverseInertiaTensor(const Matrix3 &inverseInertiaTensor) {
    this->inverseInertiaTensor = inverseInertiaTensor;
}

cyclone::real* cyclone::Rigidbody::getAngularDamping() {
    return &angularDamping;
}

void cyclone::Rigidbody::setAngularDamping(const real angularDamping) {
    this->angularDamping = angularDamping;
}

cyclone::Matrix3* cyclone::Rigidbody::getInverseInertiaTensorWorld() {
    return &inverseInertiaTensorWorld;
}

void cyclone::Rigidbody::setInverseInertiaTensorWorld(const Matrix3 &inverseInertiaTensorWorld) {
    this->inverseInertiaTensorWorld = inverseInertiaTensorWorld;
}

cyclone::Matrix4* cyclone::Rigidbody::getTransformMatrix() {
    return &transformMatrix;
}

void cyclone::Rigidbody::integrate(const real deltaTime) {

    Vector3 accel = acceleration; //gravity
    accel += forceAccum * inverseMass;
    velocity += accel * deltaTime;
    velocity *= pow(linearDamping, deltaTime);
    position.addScaledVector(velocity, deltaTime);

    const Vector3 angularAcceleration = inverseInertiaTensorWorld.transform(torqueAccum);
    angularVelocity += angularAcceleration * deltaTime;
    angularVelocity *= pow(angularDamping, deltaTime);
    orientation.addScaledVector(angularVelocity, deltaTime);
    calculateDerivedData();
    clearAccumulators();

}

void cyclone::Rigidbody::addImpulse(const Vector3 &impulse) {
    velocity += impulse;
}

void cyclone::Rigidbody::addForce(const Vector3 &force) {
    forceAccum += force;
}

void cyclone::Rigidbody::addTorque(const Vector3 &torque) {
    torqueAccum += torque;
}

void cyclone::Rigidbody::addForceAtPoint(const Vector3 &force, const Vector3 &point) {
    Vector3 pt = point;
    pt -= position;

    forceAccum += force;
    torqueAccum += pt % force;
}

void cyclone::Rigidbody::setMass(const real mass) {
    setInverseMass(1 / mass);
}

void cyclone::Rigidbody::setInertiaTensor(const Matrix3 &inertiaTensor) {
    inverseInertiaTensor.setInverse(inertiaTensor);
}

void cyclone::Rigidbody::clearAccumulators() {
    forceAccum.clear();
    torqueAccum.clear();
}



void cyclone::Rigidbody::calculateDerivedData() {

    orientation.normalise();

    _calculateTransformMatrix(transformMatrix, position, orientation);
    _transformInertiaTensor(inverseInertiaTensorWorld, orientation, dynamic_cast<const Matrix3 &&>(inverseInertiaTensor), transformMatrix);


}

cyclone::Vector3* cyclone::Rigidbody::getForceAccum() {
    return &forceAccum;
}

void cyclone::Rigidbody::setForceAccum(const Vector3 &force_accum) {
    this->forceAccum = force_accum;
}

cyclone::Rigidbody::Rigidbody()  {
    position = Vector3(0, 0, 0);
    velocity = Vector3(0, 0, 0);
    acceleration = Vector3(0, 0, 0);
    linearDamping = 1;
    inverseMass = 1;
    orientation = Quaternion(1, 0, 0, 0);
    angularVelocity = Vector3(0, 0, 0);
    torqueAccum = Vector3(0, 0, 0);
    angularDamping = 1;
    inverseInertiaTensor = Matrix3();
    inverseInertiaTensorWorld = Matrix3();
    transformMatrix = Matrix4();
}
