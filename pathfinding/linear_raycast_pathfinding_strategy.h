#pragma once
#include "pathfinding_strategy_base.h"
class LinearRaycastPathfindingStrategy : public PathfindingStrategyBase {
public:
	std::vector<std::shared_ptr<Node>> solve(Map& map, std::shared_ptr<Node> start, std::shared_ptr<Node> goal, double wind_angle_rad = 0, double no_go_angle_rad = 0);
};