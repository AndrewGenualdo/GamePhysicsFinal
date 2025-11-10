//
// Created by andrew.genualdo on 9/18/2025.
//

#ifndef RIGIDBODY_H
#define RIGIDBODY_H
#include "core.h"

namespace cyclone {
    class Rigidbody {
        Vector3 position;
        Vector3 velocity;
        Vector3 acceleration;
        real linearDamping;
        real inverseMass;
        Quaternion orientation;
        Vector3 forceAccum;
        Vector3 angularVelocity;
        Vector3 torqueAccum;
        Matrix3 inverseInertiaTensor;
        real angularDamping;
        Matrix3 inverseInertiaTensorWorld;
        Matrix4 transformMatrix;

    public:

        Rigidbody();

        [[nodiscard]] Vector3* getPosition();
        [[nodiscard]] Vector3* getVelocity();
        [[nodiscard]] Vector3* getAcceleration();
        [[nodiscard]] real* getLinearDamping();
        [[nodiscard]] real* getInverseMass();
        [[nodiscard]] Quaternion* getOrientation();
        [[nodiscard]] Vector3* getForceAccum();
        [[nodiscard]] Vector3* getAngularVelocity();
        [[nodiscard]] Vector3* getTorqueAccum();
        [[nodiscard]] Matrix3* getInverseInertiaTensor();
        [[nodiscard]] real* getAngularDamping();
        [[nodiscard]] Matrix3* getInverseInertiaTensorWorld();
        [[nodiscard]] Matrix4* getTransformMatrix();

        void setPosition(const Vector3 &position);
        void setVelocity(const Vector3 &velocity);
        void setAcceleration(const Vector3 &acceleration);
        void setLinearDamping(real linearDamping);
        void setInverseMass(real inverseMass);
        void setOrientation(const Quaternion &orientation);
        void setForceAccum(const Vector3 &force_accum);
        void setAngularVelocity(const Vector3 &angularVelocity);
        void setTorqueAccum(const Vector3 &torqueAccum);
        void setInverseInertiaTensor(const Matrix3 &inverseInertiaTensor);
        void setAngularDamping(real angularDamping);
        void setInverseInertiaTensorWorld(const Matrix3 &inverseInertiaTensorWorld);

        void integrate(real deltaTime);
        void addImpulse(const Vector3 &impulse);
        void addForce(const Vector3& force);
        void addTorque(const Vector3& torque);
        void addForceAtPoint(const Vector3& force, const Vector3& point);
        void setMass(real mass);
        void setInertiaTensor(const Matrix3 &inertiaTensor);

    private:
        //clears forceAccum and torqueAccum
        void clearAccumulators();
        //updates mat4 transform matrix
        void calculateDerivedData();

        /**
 * Inline function that creates a transform matrix from a
 * position and orientation.
 */
        static inline void _calculateTransformMatrix(Matrix4 &transformMatrix,const Vector3 &position, const Quaternion &orientation) {
            transformMatrix.data[0] = 1-2*orientation.j*orientation.j -2*orientation.k*orientation.k;
            transformMatrix.data[1] = 2*orientation.i*orientation.j -2*orientation.r*orientation.k;
            transformMatrix.data[2] = 2*orientation.i*orientation.k +2*orientation.r*orientation.j;
            transformMatrix.data[3] = position.x;transformMatrix.data[4] = 2*orientation.i*orientation.j +2*orientation.r*orientation.k;
            transformMatrix.data[5] = 1-2*orientation.i*orientation.i -2*orientation.k*orientation.k;
            transformMatrix.data[6] = 2*orientation.j*orientation.k -2*orientation.r*orientation.i;
            transformMatrix.data[7] = position.y;transformMatrix.data[8] = 2*orientation.i*orientation.k -2*orientation.r*orientation.j;
            transformMatrix.data[9] = 2*orientation.j*orientation.k +2*orientation.r*orientation.i;
            transformMatrix.data[10] = 1-2*orientation.i*orientation.i -2*orientation.j*orientation.j;
            transformMatrix.data[11] = position.z;
        }

        static inline void _transformInertiaTensor(Matrix3 &iitWorld, const Quaternion &q, const Matrix3 && iitBody, const Matrix4 &rotmat) {
            real t4 = rotmat.data[0]*iitBody.data[0]+
                rotmat.data[1]*iitBody.data[3]+
                rotmat.data[2]*iitBody.data[6];
            real t9 = rotmat.data[0]*iitBody.data[1]+
                rotmat.data[1]*iitBody.data[4]+
                rotmat.data[2]*iitBody.data[7];
            real t14 = rotmat.data[0]*iitBody.data[2]+
                rotmat.data[1]*iitBody.data[5]+
                rotmat.data[2]*iitBody.data[8];
            real t28 = rotmat.data[4]*iitBody.data[0]+
                rotmat.data[5]*iitBody.data[3]+
                rotmat.data[6]*iitBody.data[6];
            real t33 = rotmat.data[4]*iitBody.data[1]+
                rotmat.data[5]*iitBody.data[4]+
                rotmat.data[6]*iitBody.data[7];
            real t38 = rotmat.data[4]*iitBody.data[2]+
                rotmat.data[5]*iitBody.data[5]+
                rotmat.data[6]*iitBody.data[8];
            real t52 = rotmat.data[8]*iitBody.data[0]+
                rotmat.data[9]*iitBody.data[3]+
                rotmat.data[10]*iitBody.data[6];
            real t57 = rotmat.data[8]*iitBody.data[1]+
                rotmat.data[9]*iitBody.data[4]+
                rotmat.data[10]*iitBody.data[7];
            real t62 = rotmat.data[8]*iitBody.data[2]+
                rotmat.data[9]*iitBody.data[5]+
                rotmat.data[10]*iitBody.data[8];

            iitWorld.data[0] = t4*rotmat.data[0]+
                   t9*rotmat.data[1]+
                   t14*rotmat.data[2];
            iitWorld.data[1] = t4*rotmat.data[4]+
                   t9*rotmat.data[5]+
                   t14*rotmat.data[6];
            iitWorld.data[2] = t4*rotmat.data[8]+
                   t9*rotmat.data[9]+
                   t14*rotmat.data[10];
            iitWorld.data[3] = t28*rotmat.data[0]+
                   t33*rotmat.data[1]+
                   t38*rotmat.data[2];
            iitWorld.data[4] = t28*rotmat.data[4]+
                   t33*rotmat.data[5]+
                   t38*rotmat.data[6];
            iitWorld.data[5] = t28*rotmat.data[8]+
                   t33*rotmat.data[9]+
                   t38*rotmat.data[10];
            iitWorld.data[6] = t52*rotmat.data[0]+
                   t57*rotmat.data[1]+
                   t62*rotmat.data[2];
            iitWorld.data[7] = t52*rotmat.data[4]+
                   t57*rotmat.data[5]+
                   t62*rotmat.data[6];
            iitWorld.data[8] = t52*rotmat.data[8]+
                   t57*rotmat.data[9]+
                   t62*rotmat.data[10];
        }




    };
}



#endif //RIGIDBODY_H
