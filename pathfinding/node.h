#pragma once
#include <utility>
#include <queue>
#include <memory>
class Node {
public:
    int x, y;
    float gCost, hCost, fCost;
    std::shared_ptr<Node> parent;
    std::vector<std::shared_ptr<Node>> neighbors;
    Node(int x, int y) : x(x), y(y), gCost(INFINITY), hCost(INFINITY), fCost(INFINITY), parent(nullptr) {}
    Node() :gCost(INFINITY), hCost(INFINITY), fCost(INFINITY), parent(nullptr) {}

    void calculateFCost() {
        fCost = gCost + hCost;
    }
};

struct CompareNode {
    bool operator()(std::shared_ptr<Node> a, std::shared_ptr<Node> b) {
        return a->fCost > b->fCost;
    }
};