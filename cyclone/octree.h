//
// Created by cobble on 10/20/2025.
//

#ifndef GPR350_OCTREE_H
#define GPR350_OCTREE_H
#include <iostream>
#include <queue>

#include "core.h"
#include "precision.h"

namespace cyclone {

    template <typename T>
    class OctreeNode {
    public:
        OctreeNode *children = nullptr;
        std::vector<T*> *obj = nullptr;

        OctreeNode() = default;
        OctreeNode(const real size, real minSize) {
            if (size * 0.5f > minSize) {
                children = new OctreeNode<T>[8];
                for (int i = 0; i < 8; i++) {
                    new (&children[i]) OctreeNode(size * 0.5f, minSize);
                }
                obj = nullptr;
            } else {
                obj = new std::vector<T*>();
            }
        }

        ~OctreeNode() {
            cleanup();
        }

        void cleanup() {

            if (children != nullptr) {
                for (int i = 0; i < 8; i++) {
                    children[i].cleanup();
                    children[i].~OctreeNode();
                }
                delete [] children;
                children = nullptr;
            }

            if (obj != nullptr) {
                delete obj;
                obj = nullptr;
            }
        }

        OctreeNode(const OctreeNode&) = delete;
        OctreeNode& operator=(const OctreeNode&) = delete;

        void clearObjects() {
            if (obj != nullptr) obj->clear(); // clear this leaf's objects
            if (children != nullptr) {
                for (int i = 0; i < 8; ++i) {
                    children[i].clearObjects();
                }
            }
        }

        static bool SphereIntersectsAABB(const Vector3 &center, const real radius, const Vector3 &aabbCenter, const Vector3& aabbHalfSize)
        {
            const Vector3 min = aabbCenter - aabbHalfSize;
            const Vector3 max = aabbCenter + aabbHalfSize;

            const Vector3 clamp = center.clamp(min, max);

            const real distSq = (clamp.x - center.x) * (clamp.x - center.x) + (clamp.y - center.y) * (clamp.y - center.y) + (clamp.z - center.z) * (clamp.z - center.z);

            return distSq <= radius * radius;
        }

        OctreeNode<T>* getNode(const Vector3 position, const Vector3 offset, const real size, const real minSize) {

            if (size * 0.5f <= minSize) { //leaf node
                if (obj == nullptr) obj = new std::vector<T*>();
                return this;
            }

            int index = 0;
            Vector3 off;

            if (position.x > offset.x) { off.x += 0.25; index += 1; } else { off.x -= 0.25; }
            if (position.y > offset.y) { off.y += 0.25; index += 2; } else { off.y -= 0.25; }
            if (position.z > offset.z) { off.z += 0.25; index += 4; } else { off.z -= 0.25; }

            off *= size;

            return children[index].getNode(position, offset + off, size * 0.5f, minSize);
        }

        std::vector<std::pair<OctreeNode<T>*, Vector3>> getNodes(const Vector3 position, const real radius, const Vector3 offset, const real size, const real minSize) {
            return getNodes(position, radius, offset, offset, size, size, minSize);
        }

        std::vector<std::pair<OctreeNode<T>*, Vector3>> getNodes(const Vector3 position, const real radius, const Vector3 startOffset, const Vector3 offset, const real startSize, const real size, const real minSize) {
            if (size * 0.5f <= minSize) { //leaf node
                if (obj == nullptr) obj = new std::vector<T*>();
                std::vector<std::pair<OctreeNode<T>*, Vector3>> nodes;
                const Vector3 halfSize = Vector3(size, size, size) * 0.5f;

                const Vector3 cornerOffset = Vector3(position.x > offset.x ? 0 : -1, position.y > offset.y ? 0 : -1, position.z > offset.z ? 0 : -1);
                for (int y = 0; y < 2; y++) {
                    for (int z = 0; z < 2; z++) {
                        for (int x = 0; x < 2; x++) {
                            Vector3 neighborCenter = offset + (Vector3(x, y, z) + cornerOffset) * size;
                            if (SphereIntersectsAABB(position, radius, neighborCenter, halfSize)) {
                                nodes.emplace_back(getNode(neighborCenter, startOffset, startSize, minSize), neighborCenter);
                            }
                        }
                    }
                }

                return nodes;
            }

            int index = 0;
            Vector3 off;

            if (position.x > offset.x) { off.x += 0.25; index += 1; } else { off.x -= 0.25; }
            if (position.y > offset.y) { off.y += 0.25; index += 2; } else { off.y -= 0.25; }
            if (position.z > offset.z) { off.z += 0.25; index += 4; } else { off.z -= 0.25; }

            off *= size;

            return children[index].getNodes(position, radius, offset + off, size * 0.5f, minSize);
        }
    };



} // cyclone

#endif //GPR350_OCTREE_H