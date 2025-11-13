//
// Created by andrew.genualdo on 9/29/2025.
//

#ifndef GPR350_COLLIDER_H
#define GPR350_COLLIDER_H
#include <vector>

#include "rigidbody.h"

namespace cyclone {
    enum class ColliderType {
        None = 0,
        Sphere = 1,
        Plane = 2,
        Box = 3
    };

    class Collider {
        ColliderType type;
        Rigidbody *pBody = nullptr;
        Matrix4 offset;
        Matrix4 transform;
        int overlapCount;

    public:

        [[nodiscard]] ColliderType getType() const;
        void setType(ColliderType type);
        [[nodiscard]] Rigidbody * getRigidbody() const;
        void setRigidbody(Rigidbody * pBody);
        [[nodiscard]] int getOverlapCount() const;
        void setOverlapCount(int overlapCount);
        void updateInternals();
        [[nodiscard]] Vector3 getPosition() const;

        [[nodiscard]] Vector3 getAxis(int index) const;
        Collider();
        ~Collider();

    };

    class SphereCollider : public Collider {
        real radius;

    public:
        [[nodiscard]] real getRadius() const;
        void setRadius(real radius);
        SphereCollider();

    };

    class PlaneCollider : public Collider {
        Vector3 normal;
        real offset;
    public:
        [[nodiscard]] Vector3 getNormal() const;
        void setNormal(const Vector3 &normal);
        [[nodiscard]] real getOffset() const;
        void setOffset(real offset);
        PlaneCollider();
        PlaneCollider(Vector3 position, Vector3 normal);
    };

    class BoxCollider : public Collider {
        Vector3 halfSize;

    public:
        [[nodiscard]] Vector3 getHalfSize() const;
        void setHalfSize(const Vector3 &halfSize);
        BoxCollider();

    };

    class IntersectionTests {
    public:
        static bool SphereSphere(const SphereCollider& a, const SphereCollider& b);
        static bool SpherePlane(const SphereCollider& a, const PlaneCollider& b);
        static bool SphereBox(const SphereCollider& a, const BoxCollider& b);
        static bool PlanePlane(const PlaneCollider& a, const PlaneCollider& b);
        static bool PlaneBox(const PlaneCollider& a, const BoxCollider& b);
        static bool BoxBox(const BoxCollider& a, const BoxCollider& b);
    };

    class Contact{
    public:
        Vector3 point; //The point of contact (or an approximation)
        Vector3 normal; //The direction, pointing from B to A
        real penetration = 0; //How deeply are the objects overlapping along the normal?
        Rigidbody* body[2]{}; //The two bodies involved

        Contact();
        Contact(Rigidbody* a, Rigidbody* b);

    };

    class CollisionData{
    public:
        std::vector<Contact*> contacts;
        void reset();
        void addContacts(Contact *contact);

        void resolveAllContacts(real restitution) const;

        CollisionData() = default;
        ~CollisionData();
    };

    class CollisionTests {
    public:
        static int SphereSphere(const SphereCollider& a, const SphereCollider& b, CollisionData* data);
        static int SphereTruePlane(const SphereCollider& sphere, const PlaneCollider& plane, CollisionData* data);
        static int SphereBox(const SphereCollider& sphere, const BoxCollider& box, CollisionData* data);
    };

    class ContactResolver{
    public:
        static void resolveContacts(const std::vector<Contact*> *contacts, real restitution);
    private:
        static void resolveVelocity(const Contact *contact, real restitution);
        static void resolveInterpenetration(const Contact *contact);
    };

} // cyclone

#endif //GPR350_COLLIDER_H