#pragma once
#include "pathfinding_strategy_base.h"
#define TURN_WEIGHT 1
struct NodeComparator {
    bool operator()(const Node& a, const Node& b) const {
        extern std::unordered_map<Node, double, NodeHasher>* pFScoreMap;
        return pFScoreMap->at(a) > pFScoreMap->at(b);
    }
};
using NodePriorityQueue = std::priority_queue<Node, std::vector<Node>, NodeComparator>;

class AStarPathfindingStrategy : public PathfindingStrategyBase {
public:
	std::vector<Node> solve(Map map, Node start, Node goal, double wind_angle_rad = 0, double no_go_angle = 0);
};