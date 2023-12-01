#pragma once
#include "pathfinding_strategy_base.h"
#define TURN_WEIGHT 1

class AStarPathfindingStrategy : public PathfindingStrategyBase {
public:
	std::vector<Node*> solve(Map& map, Node* start, Node* goal, double wind_angle_rad = 0, double no_go_angle = 0);
};