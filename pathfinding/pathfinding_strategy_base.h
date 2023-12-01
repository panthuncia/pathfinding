#pragma once
#include <vector>
#include "map.h"
#include "node.h"
class PathfindingStrategyBase {
	virtual std::vector<Node*> solve(Map& map, Node* start, Node* goal, double wind_angle_rad, double no_go_angle) = 0;
};