#pragma once
#include <vector>
#include "map.h"
#include "node.h"
class PathfindingStrategyBase {
	virtual std::vector<std::shared_ptr<Node>> solve(Map& map, std::shared_ptr<Node> start, std::shared_ptr<Node> goal, double wind_angle_rad, double no_go_angle) = 0;
};