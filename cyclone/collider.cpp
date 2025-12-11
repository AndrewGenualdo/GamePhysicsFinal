//
// Created by andrew.genualdo on 9/29/2025.
//

#include "collider.h"

#include <algorithm>
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
        pBody = new Rigidbody();
        overlapCount = 0;
        type = ColliderType::None;
        offset = Matrix4();
        transform = *pBody->getTransformMatrix();

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

    SphereCollider::SphereCollider() {
        this->radius = 1;
        this->setType(ColliderType::Sphere);
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
        getRigidbody()->setPosition(position);
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

    bool IntersectionTests::SphereSphere(const Collider &a, const Collider &b) {
        const real aRadius = reinterpret_cast<const SphereCollider &>(a).getRadius();
        const real bRadius = reinterpret_cast<const SphereCollider &>(b).getRadius();
        return (a.getPosition() - b.getPosition()).squareMagnitude() < (aRadius + bRadius) * (aRadius + bRadius);
    }

    bool IntersectionTests::SpherePlane(const Collider &a, const Collider &b) {
        const SphereCollider &sphere = reinterpret_cast<const SphereCollider &>(a);
        const PlaneCollider &plane = reinterpret_cast<const PlaneCollider &>(b);
        return std::abs(plane.getNormal() * a.getPosition() - plane.getOffset()) <= sphere.getRadius();
    }

    bool IntersectionTests::SphereBox(const Collider &a, const Collider &b) {
        const SphereCollider &sphere = reinterpret_cast<const SphereCollider &>(a);
        const BoxCollider &box = reinterpret_cast<const BoxCollider &>(b);

        const Vector3 p = b.getRigidbody()->getTransformMatrix()->transformInverse(*a.getRigidbody()->getPosition());
        const Vector3 nearest = p.clamp(-box.getHalfSize() * 0.5f, box.getHalfSize() * 0.5f);
        const real d = (nearest - p).squareMagnitude();
        return d <= sphere.getRadius() * sphere.getRadius();
    }

    bool IntersectionTests::PlanePlane(const Collider &a, const Collider &b) {
        return false;
    }

    real sizeAlongAxis(const BoxCollider& box, const Vector3 &axis) {
        const real dx = box.getHalfSize().x * abs(axis * box.getAxis(0));
        const real dy = box.getHalfSize().y * abs(axis * box.getAxis(1));
        const real dz = box.getHalfSize().z * abs(axis * box.getAxis(2));
        return dx + dy + dz;
    }

    bool IntersectionTests::PlaneBox(const Collider &a, const Collider &b) {
        const PlaneCollider &plane = reinterpret_cast<const PlaneCollider &>(a);
        const BoxCollider &box = reinterpret_cast<const BoxCollider &>(b);
        return sizeAlongAxis(box, plane.getNormal()) > plane.getNormal() * *box.getRigidbody()->getPosition() - (plane.getOffset());
        //return false;

    }

    bool overlapOnAxis(const BoxCollider &a, const BoxCollider &b, const Vector3 &axis, const Vector3 &toCenter) {
            if(axis.squareMagnitude() < 0.01) return true; //close enough
            const real aProject = sizeAlongAxis(a, axis);
            const real bProject = sizeAlongAxis(b, axis);
            const real distance = real_abs(toCenter * axis);
            return distance <= aProject + bProject;
    }

    bool IntersectionTests::BoxBox(const Collider &a, const Collider &b) {
        const BoxCollider &af = reinterpret_cast<const BoxCollider &>(a);
        const BoxCollider &bf = reinterpret_cast<const BoxCollider &>(b);
        const Vector3 toCenter = a.getAxis(3) - b.getAxis(3);
        #define TEST_OVERLAP(axis) overlapOnAxis(af, bf, (axis), toCenter)
        return TEST_OVERLAP(af.getAxis(0)) && // a's face normals
               TEST_OVERLAP(af.getAxis(1)) &&
               TEST_OVERLAP(af.getAxis(2)) &&
               TEST_OVERLAP(bf.getAxis(0)) && // b's face noramls
               TEST_OVERLAP(bf.getAxis(1)) &&
               TEST_OVERLAP(bf.getAxis(2)) &&
               TEST_OVERLAP(af.getAxis(0) % bf.getAxis(0)) && // cross products between edges - 3 unique edge directions per cube (so nine tests)
               TEST_OVERLAP(af.getAxis(0) % bf.getAxis(1)) &&
               TEST_OVERLAP(af.getAxis(0) % bf.getAxis(2)) &&
               TEST_OVERLAP(af.getAxis(1) % bf.getAxis(0)) &&
               TEST_OVERLAP(af.getAxis(1) % bf.getAxis(1)) &&
               TEST_OVERLAP(af.getAxis(1) % bf.getAxis(2)) &&
               TEST_OVERLAP(af.getAxis(2) % bf.getAxis(0)) &&
               TEST_OVERLAP(af.getAxis(2) % bf.getAxis(1)) &&
               TEST_OVERLAP(af.getAxis(2) % bf.getAxis(2));
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

    int CollisionTests::SphereSphere(const Collider &a, const Collider &b, CollisionData *data) {
        const SphereCollider &af = reinterpret_cast<const SphereCollider &>(a);
        const SphereCollider &bf = reinterpret_cast<const SphereCollider &>(b);

        Contact *contact = new Contact();
        contact->body[0] = a.getRigidbody();
        contact->body[1] = b.getRigidbody();
        const real distance = (a.getPosition() - b.getPosition()).magnitude();
        contact->penetration = distance - (af.getRadius() + bf.getRadius());
        contact->point = a.getPosition() + (b.getPosition() - a.getPosition()) * 0.5f;
        contact->normal = (b.getPosition() - a.getPosition());
        contact->normal.normalise();
        data->addContacts(contact);
        return 1;
    }

    int CollisionTests::SphereTruePlane(const Collider &a, const Collider &b, CollisionData *data) {
        const SphereCollider &sphere = reinterpret_cast<const SphereCollider &>(a);
        const PlaneCollider &plane = reinterpret_cast<const PlaneCollider &>(b);

        Contact *contact = new Contact();
        contact->body[0] = a.getRigidbody();
        contact->body[1] = b.getRigidbody();
        const real distance = (plane.getNormal() * (a.getPosition() - b.getPosition())) - sphere.getRadius();
        Vector3 normal = plane.getNormal();
        real penetration = -distance;
        if (distance > 0) {
            normal = -normal;
            penetration = -penetration;
        }
        contact->normal = normal;
        contact->penetration = penetration;
        contact->point = a.getPosition() - normal * sphere.getRadius();
        data->addContacts(contact);
        return 1;
    }

    int CollisionTests::SphereBox(const Collider &a, const Collider &b, CollisionData *data) {
        const SphereCollider &sphere = reinterpret_cast<const SphereCollider &>(a);
        const BoxCollider &box = reinterpret_cast<const BoxCollider &>(b);


        //sphere center ->box local space
        const Vector3 relCenter = b.getRigidbody()->getTransformMatrix()->transformInverse(*a.getRigidbody()->getPosition());

        //gets point box closest to sphere center
        const Vector3 closestPoint = relCenter.clamp(-box.getHalfSize() * 0.5f, box.getHalfSize() * 0.5f);

        //vector from closest point to sphere center
        const Vector3 localDelta = relCenter - closestPoint;

        Contact *contact = new Contact();
        contact->body[0] = a.getRigidbody();
        contact->body[1] = b.getRigidbody();

        Vector3 normal;
        real distance = localDelta.magnitude();

        //if sphere center is inside box or not
        if (distance > 0.0001f) { //not in box
            //normal to world space
            normal = b.getRigidbody()->getTransformMatrix()->transformDirection(localDelta.unit());
            normal.normalise();


        } else { //in box
            const Vector3 boxCenter = *b.getRigidbody()->getPosition();
            normal = *a.getRigidbody()->getPosition() - boxCenter;
            if (normal.squareMagnitude() < 0.0001f)
                normal = Vector3(1, 0, 0); //arbitrary default direction
            normal.normalise();
            distance = 0.0f;
        }

        //contact point in world space
        //closest point transformed to world space + normal scaled by penetration
        contact->point = b.getRigidbody()->getTransformMatrix()->transform(closestPoint);;
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
        Vector3 rA = contact->point - *contact->body[0]->getPosition();
        Vector3 rB = contact->point - *contact->body[1]->getPosition();

        Vector3 vA = *contact->body[0]->getVelocity() + (contact->body[0]->getAngularVelocity()->vectorProduct(rA));
        Vector3 vB = *contact->body[1]->getVelocity() + (contact->body[1]->getAngularVelocity()->vectorProduct(rB));

        Vector3 dv = vA - vB;

        real massComponent = (  *contact->body[0]->getInverseMass() + *contact->body[1]->getInverseMass()) +
                                contact->normal.scalarProduct((*contact->body[0]->getInverseInertiaTensor() * (rA.vectorProduct(contact->normal))).vectorProduct(rA)) +
                                contact->normal.scalarProduct((*contact->body[1]->getInverseInertiaTensor() * (rB.vectorProduct(contact->normal))).vectorProduct(rB));
        if(massComponent > 0)
        {

            real J = std::max(-dv.scalarProduct(contact->normal) / massComponent, 0.0f);

            if(*contact->body[0]->getInverseMass() > 0)
            {
                contact->body[0]->addVelocity(-(contact->normal * (J * (*contact->body[0]->getInverseMass()))));
                contact->body[0]->addRotation(-(*contact->body[0]->getInverseInertiaTensor() * (rA.vectorProduct(contact->normal * J))));
            }
            if(*contact->body[1]->getInverseMass() > 0)
            {
                contact->body[1]->addVelocity((contact->normal * (J * (*contact->body[1]->getInverseMass()))));
                contact->body[1]->addRotation((*contact->body[1]->getInverseInertiaTensor() * (rB.vectorProduct(contact->normal * J))));
            }
        }

        Vector3 tangent = dv - contact->normal * (dv.scalarProduct(contact->normal));
        real tangentMag = tangent.magnitude();
        if(tangentMag > 0.0001f)
        {
            tangent.normalise();
            //float frictionalMass = ( pnodeA->GetInverseMass () + pnodeB->GetInverseMass()) +Vector3::Dot(tangent, Vector3::Cross(pnodeA->GetInverseInertia() * Vector3::Cross(r1, tangent), r1) + Vector3::Cross(pnodeB->GetInverseInertia() * Vector3::Cross(r2, tangent), r2));
            real frictionalMass = (*contact->body[0]->getInverseMass() + *contact->body[1]->getInverseMass()) +
                tangent.scalarProduct((*contact->body[0]->getInverseInertiaTensor() * (rA.vectorProduct(tangent))).vectorProduct(rA)) +
                    tangent.scalarProduct((*contact->body[1]->getInverseInertiaTensor() * (rB.vectorProduct(tangent))).vectorProduct(rB));
            if(frictionalMass > 0)
            {
                real frictionCoeff = 1.0f; //hardcoded for now
                real Jt = -dv.scalarProduct(tangent) * frictionCoeff / frictionalMass;
                if(*contact->body[0]->getInverseMass() > 0)
                {
                    contact->body[0]->addVelocity(-(tangent * (Jt * (*contact->body[0]->getInverseMass()))));
                    contact->body[0]->addRotation(-(*contact->body[0]->getInverseInertiaTensor() * (rA.vectorProduct(tangent * Jt))));
                }
                if(*contact->body[1]->getInverseMass() > 0)
                {
                    contact->body[1]->addVelocity(tangent * (Jt * (*contact->body[1]->getInverseMass())));
                    contact->body[1]->addRotation((*contact->body[1]->getInverseInertiaTensor() * (rB.vectorProduct(tangent * Jt))));
                }
            }
        }




    }

    void ContactResolver::resolveInterpenetration(const Contact *contact) {
        const Vector3 displacementA = (contact->normal * contact->penetration) * (*contact->body[0]->getInverseMass() / (*contact->body[0]->getInverseMass() + *contact->body[1]->getInverseMass()));
        const Vector3 displacementB = -(contact->normal * contact->penetration) * (*contact->body[1]->getInverseMass() / (*contact->body[0]->getInverseMass() + *contact->body[1]->getInverseMass()));
        contact->body[0]->setPosition(*contact->body[0]->getPosition() + displacementA);
        contact->body[1]->setPosition(*contact->body[1]->getPosition() + displacementB);
    }
} // cyclone