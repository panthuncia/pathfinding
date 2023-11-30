#pragma once
#include <utility>
#include <queue>
#include <unordered_map>
class Node {
public:
    int x, y;
    bool operator==(const Node& other) const {
        return x == other.x && y == other.y;
    }
};

struct NodeHasher {
    std::size_t operator()(const Node& k) const {
        std::size_t h1 = std::hash<int>()(k.x);
        std::size_t h2 = std::hash<int>()(k.y);

        return h1 ^ h2;
    }
};

using NodeUnorderedMap = std::unordered_map<Node, Node, NodeHasher>;
