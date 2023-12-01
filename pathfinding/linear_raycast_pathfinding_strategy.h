#pragma once
#include "pathfinding_strategy_base.h"
class LinearRaycastPathfindingStrategy : public PathfindingStrategyBase {
public:
	std::vector<Node*> solve(Map& map, Node* start, Node* goal, double wind_angle_rad = 0, double no_go_angle_rad = 0);
};