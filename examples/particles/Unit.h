//
// Created by ehosack on 11/13/25.
//

#ifndef UNIT_H
#define UNIT_H
#include "raylib.h"
#include "cyclone/collider.h"
#include "cyclone/cyclone.h"
#include "cyclone/rigidbody.h"

namespace bowling {

class Unit {
    friend class UnitManager;
protected:
    cyclone::Rigidbody* rigidbody;
    cyclone::Collider* collider;
    Model* model;
    bool active;
    bool visible;
public:
    Unit();
    void update(double dt);
    void draw(double dt, Color meshColor, Color wireframeColor);

    cyclone::Rigidbody* getRigidbody() { return pRigidbody; }
    cyclone::Collider* getCollider() { return collider; }
    Model* getModel() { return model; }

    void setRigidbody(cyclone::Rigidbody* rb) { pRigidbody = rb; }
    void setCollider(cyclone::Collider* col) { collider = col; }
    void setModel(Model* m) { model = m; }


};

} // bowling

#endif //UNIT_H
