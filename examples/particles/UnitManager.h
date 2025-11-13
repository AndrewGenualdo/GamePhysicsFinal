//
// Created by ehosack on 11/13/25.
//

#ifndef UNITMANAGER_H
#define UNITMANAGER_H
#include <vector>

#include "raylib.h"
#include "cyclone/collider.h"
#include "cyclone/rigidbody.h"


namespace bowling
{
    class Unit;

    class UnitManager {
        friend class bowling::Unit;
    protected:
        std::vector<Unit*> units;
    public:
        UnitManager(int maxUnits = 0);
        ~UnitManager();
        void update(double dt);
        void draw(double dt, bool debug);
        Unit* createUnit(cyclone::Rigidbody* rb = nullptr, cyclone::Collider* col = nullptr, Model* model = nullptr, bool active = true, bool visible = true);
        void destroyUnit(Unit* unit);
        void hideUnit(Unit* unit);
        void showUnit(Unit* unit);
        void freezeUnit(Unit* unit);
        void thawUnit(Unit* unit);


    };
}


#endif //UNITMANAGER_H
