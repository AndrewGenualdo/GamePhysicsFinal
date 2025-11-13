//
// Created by andrew.genualdo on 9/29/2025.
//

#include "collider.h"

#include <iostream>


namespace cyclone {
    int Collider::getOverlapCount() const {
        return overlapCount;
    }

    void Collider::setOverlapCount(const int overlapCount) {
        this->overlapCount = overlapCount;
    }

    void Collider::updateInternals() {
        transform = *pBody->getTransformMatrix() * offset;
    }

    Vector3 Collider::getPosition() const {
        return *getRigidbody()->getPosition();
    }

    ColliderType Collider::getType() const {
        return type;
    }

    void Collider::setType(const ColliderType type) {
        this->type = type;
    }

    Rigidbody * Collider::getRigidbody() const {
        return pBody;
    }

    void Collider::setRigidbody(Rigidbody * const pBody) {
        this->pBody = pBody;
    }

    Vector3 Collider::getAxis(const int index) const {
        return transform.getAxisVector(index);
    }

    Collider::Collider() {
        overlapCount = 0;
        type = ColliderType::None;
        offset = Matrix4();

    }

    Collider::~Collider() {
        if (pBody != nullptr) {
            delete pBody;
            pBody = nullptr;
        }

    }

    real SphereCollider::getRadius() const {
        return radius;
    }

    void SphereCollider::setRadius(const real radius) {
        this->radius = radius;
    }

    SphereCollider::SphereCollider() : Collider() {
        this->radius = 1;
    }

    Vector3 PlaneCollider::getNormal() const {
        return normal;
    }

    void PlaneCollider::setNormal(const Vector3 &normal) {
        this->normal = normal;
    }

    real PlaneCollider::getOffset() const {
        return offset;
    }

    void PlaneCollider::setOffset(const real offset) {
        this->offset = offset;
    }

    PlaneCollider::PlaneCollider() {
        this->normal = Vector3(0, 1, 0);
        this->offset = normal * getPosition();
        this->setType(ColliderType::Plane);
        this->getRigidbody()->setInverseMass(0.0f);
    }

    PlaneCollider::PlaneCollider(const Vector3 position, const Vector3 normal) {
        this->offset = normal * position;
        this->normal = normal;
        this->setType(ColliderType::Plane);
        this->getRigidbody()->setInverseMass(0.0f);
    }

    Vector3 BoxCollider::getHalfSize() const {
        return halfSize;
    }

    void BoxCollider::setHalfSize(const Vector3 &halfSize) {
        this->halfSize = halfSize;
    }

    BoxCollider::BoxCollider() : Collider() {
        this->halfSize = Vector3(0.5, 0.5, 0.5);
        this->setType(ColliderType::Box);
    }

    bool IntersectionTests::SphereSphere(const SphereCollider &a, const SphereCollider &b) {
        return (a.getPosition() - b.getPosition()).squareMagnitude() < (a.getRadius() + b.getRadius()) * (a.getRadius() + b.getRadius());
    }

    bool IntersectionTests::SpherePlane(const SphereCollider &a, const PlaneCollider &b) {
        //return a.getRadius() > b.getNormal() * *a.getRigidbody()->getPosition() - b.getOffset();
        //return a.getRadius() > abs(b.getNormal() * (*a.getRigidbody()->getPosition() - (b.getAxis(0) * b.getOffset())));
        return std::abs(b.getNormal() * a.getPosition() - b.getOffset()) <= a.getRadius();
    }

    bool IntersectionTests::SphereBox(const SphereCollider &a, const BoxCollider &b) {
        const Vector3 p = b.getRigidbody()->getTransformMatrix()->transformInverse(*a.getRigidbody()->getPosition());
        const Vector3 nearest = p.clamp(-b.getHalfSize(), b.getHalfSize());
        const real d = (nearest - p).squareMagnitude();
        return d <= a.getRadius() * a.getRadius();
    }

    bool IntersectionTests::PlanePlane(const PlaneCollider &a, const PlaneCollider &b) {
        return false;
    }

    real sizeAlongAxis(const BoxCollider& box, const Vector3 &axis) {
        const real dx = box.getHalfSize().x * abs(axis * box.getAxis(0));
        const real dy = box.getHalfSize().y * abs(axis * box.getAxis(1));
        const real dz = box.getHalfSize().z * abs(axis * box.getAxis(2));
        return dx + dy + dz;
    }

    bool IntersectionTests::PlaneBox(const PlaneCollider &a, const BoxCollider &b) {
        return sizeAlongAxis(b, a.getNormal()) > a.getNormal() * *b.getRigidbody()->getPosition() - (a.getOffset());
        return false;

    }

    bool overlapOnAxis(const BoxCollider &a, const BoxCollider &b, const Vector3 &axis, const Vector3 &toCenter) {
            if(axis.squareMagnitude() < 0.01) return true; //close enough
            const real aProject = sizeAlongAxis(a, axis);
            const real bProject = sizeAlongAxis(b, axis);
            const real distance = real_abs(toCenter * axis);
            return distance <= aProject + bProject;
    }

    bool IntersectionTests::BoxBox(const BoxCollider &a, const BoxCollider &b) {
        const Vector3 toCenter = a.getAxis(3) - b.getAxis(3);
        #define TEST_OVERLAP(axis) overlapOnAxis(a, b, (axis), toCenter)
        return TEST_OVERLAP(a.getAxis(0)) && // a's face normals
               TEST_OVERLAP(a.getAxis(1)) &&
               TEST_OVERLAP(a.getAxis(2)) &&
               TEST_OVERLAP(b.getAxis(0)) && // b's face noramls
               TEST_OVERLAP(b.getAxis(1)) &&
               TEST_OVERLAP(b.getAxis(2)) &&
               TEST_OVERLAP(a.getAxis(0) % b.getAxis(0)) && // cross products between edges - 3 unique edge directions per cube (so nine tests)
               TEST_OVERLAP(a.getAxis(0) % b.getAxis(1)) &&
               TEST_OVERLAP(a.getAxis(0) % b.getAxis(2)) &&
               TEST_OVERLAP(a.getAxis(1) % b.getAxis(0)) &&
               TEST_OVERLAP(a.getAxis(1) % b.getAxis(1)) &&
               TEST_OVERLAP(a.getAxis(1) % b.getAxis(2)) &&
               TEST_OVERLAP(a.getAxis(2) % b.getAxis(0)) &&
               TEST_OVERLAP(a.getAxis(2) % b.getAxis(1)) &&
               TEST_OVERLAP(a.getAxis(2) % b.getAxis(2));
    }

    Contact::Contact() {
        this->penetration = 0;
    }

    Contact::Contact(Rigidbody* a, Rigidbody* b) {
        this->body[0] = a;
        this->body[1] = b;
    }

    void CollisionData::reset() {
        for (const auto & contact : contacts) {
            delete contact;
        }
        contacts.clear();
    }

    void CollisionData::addContacts(Contact *contact) {
        contacts.push_back(contact);
    }

    void CollisionData::resolveAllContacts(const real restitution) const {
        ContactResolver::resolveContacts(&contacts, restitution);
    }

    CollisionData::~CollisionData() {
        for (const auto & contact : contacts) {
            delete contact;
        }
    }

    int CollisionTests::SphereSphere(const SphereCollider &a, const SphereCollider &b, CollisionData *data) {

        Contact *contact = new Contact();
        contact->body[0] = a.getRigidbody();
        contact->body[1] = b.getRigidbody();
        const real distance = (a.getPosition() - b.getPosition()).magnitude();
        contact->penetration = distance - (a.getRadius() + b.getRadius());
        contact->point = a.getPosition() + (b.getPosition() - a.getPosition()) * 0.5f;
        contact->normal = (b.getPosition() - a.getPosition());
        contact->normal.normalise();
        data->addContacts(contact);
        return 1;
    }

    int CollisionTests::SphereTruePlane(const SphereCollider &sphere, const PlaneCollider &plane, CollisionData *data) {
        Contact *contact = new Contact();
        contact->body[0] = sphere.getRigidbody();
        contact->body[1] = plane.getRigidbody();
        const real distance = (plane.getNormal() * (sphere.getPosition() - plane.getPosition())) - sphere.getRadius();
        Vector3 normal = plane.getNormal();
        real penetration = -distance;
        if (distance > 0) {
            normal = -normal;
            penetration = -penetration;
        }
        contact->normal = normal;
        contact->penetration = penetration;
        contact->point = sphere.getPosition() - normal * sphere.getRadius();
        data->addContacts(contact);
        return 1;
    }

    int CollisionTests::SphereBox(const SphereCollider &sphere, const BoxCollider &box, CollisionData *data) {
        //sphere center -> box local space
        const Vector3 relCenter = box.getRigidbody()->getTransformMatrix()->transformInverse(*sphere.getRigidbody()->getPosition());

        //gets point box closest to sphere center
        const Vector3 closestPoint = relCenter.clamp(-box.getHalfSize(), box.getHalfSize());

        //vector from closest point to sphere center
        const Vector3 localDelta = relCenter - closestPoint;

        Contact *contact = new Contact();
        contact->body[0] = sphere.getRigidbody();
        contact->body[1] = box.getRigidbody();

        Vector3 normal;
        real distance = localDelta.magnitude();

        //if sphere center is inside box or not
        if (distance > 0.0001f) { //not in box
            //normal to world space
            normal = box.getRigidbody()->getTransformMatrix()->transformDirection(localDelta * (1.0f / distance));
            normal.normalise();

        } else { //in box
            const Vector3 boxCenter = *box.getRigidbody()->getPosition();
            normal = (*sphere.getRigidbody()->getPosition() - boxCenter);
            if (normal.squareMagnitude() < 0.0001f)
                normal = Vector3(1, 0, 0); //arbitrary default direction
            normal.normalise();
            distance = 0.0f;
        }

        //contact point in world space
        //closest point transformed to world space + normal scaled by penetration
        contact->point = box.getRigidbody()->getTransformMatrix()->transform(closestPoint);;
        contact->normal = normal;
        contact->penetration = sphere.getRadius() - distance;

        // Add the contact to the data
        data->addContacts(contact);
        return 1;
    }


    void ContactResolver::resolveContacts(const std::vector<Contact*> *contacts, const real restitution) {
        for (const auto contact : *contacts) {
            resolveVelocity(contact, restitution);
            resolveInterpenetration(contact);
        }
    }

    void ContactResolver::resolveVelocity(const Contact *contact, const real restitution) {
        const Vector3 relVel = *contact->body[0]->getVelocity() - *contact->body[1]->getVelocity();
        const real sepVel = relVel * contact->normal;
        const real newSepVel = -restitution * sepVel;
        const real deltaVel = newSepVel - sepVel;
        const Vector3 j = contact->normal * (deltaVel / (*contact->body[0]->getInverseMass() + *contact->body[1]->getInverseMass()));
        const Vector3 deltaA = j * *contact->body[0]->getInverseMass();
        const Vector3 deltaB = -j * *contact->body[1]->getInverseMass();
        contact->body[0]->addImpulse(deltaA);
        contact->body[1]->addImpulse(deltaB);
    }

    void ContactResolver::resolveInterpenetration(const Contact *contact) {
        const Vector3 displacementA = (contact->normal * contact->penetration) * (*contact->body[0]->getInverseMass() / (*contact->body[0]->getInverseMass() + *contact->body[1]->getInverseMass()));
        const Vector3 displacementB = -(contact->normal * contact->penetration) * (*contact->body[1]->getInverseMass() / (*contact->body[0]->getInverseMass() + *contact->body[1]->getInverseMass()));
        contact->body[0]->setPosition(*contact->body[0]->getPosition() + displacementA);
        contact->body[1]->setPosition(*contact->body[1]->getPosition() + displacementB);
    }
} // cyclone