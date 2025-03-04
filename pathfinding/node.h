#pragma once
#include <utility>
#include <queue>

class Node {
public:
    float x, y;
    float gCost, hCost, fCost;
    Node* parent;
    std::vector<Node*> neighbors;
    Node(float x, float y) : x(x), y(y), gCost(INFINITY), hCost(INFINITY), fCost(INFINITY), parent(nullptr) {}
    Node() :gCost(INFINITY), hCost(INFINITY), fCost(INFINITY), parent(nullptr) {}
    void reset()
    {
        gCost = INFINITY;
        hCost = INFINITY;
        fCost = INFINITY;
        parent = nullptr;
    }
    void calculateFCost() {
        fCost = gCost + hCost;
    }
    bool operator == (const Node& n) {
        if (x == n.x && y == n.y) {
            return true;
        }
        return false;
    }
};

struct PointKey {
    float x, y;

    bool operator==(const PointKey& other) const {
        return x == other.x && y == other.y;
    }
};

namespace std {
    template <>
    struct hash<PointKey> {
        std::size_t operator()(const PointKey& k) const {
            return ((std::hash<float>()(k.x) ^ (std::hash<float>()(k.y) << 1)) >> 1);
        }
    };
}

struct CompareNode {
    bool operator()(Node* a, Node* b) {
        return a->fCost > b->fCost;
    }
};