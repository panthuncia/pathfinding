#include "linear_raycast_pathfinding_strategy.h"
#include <iostream>
#define _USE_MATH_DEFINES
#include "math.h"
#include "raycast.h"

std::vector<std::shared_ptr<Node>> LinearRaycastPathfindingStrategy::solve(Map& map, std::shared_ptr<Node> start, std::shared_ptr<Node> goal, double wind_angle_rad, double no_go_angle_rad) {
	if (raycast(map, start->x, start->y, goal->x, goal->y)) {
		return { start, goal };
	}
	return {};
}