#include "linear_raycast_pathfinding_strategy.h"
#include <iostream>
#define _USE_MATH_DEFINES
#include "math.h"
#include "raycast.h"

std::vector<Node*> LinearRaycastPathfindingStrategy::solve(Map& map, Node* start, Node* goal, double wind_angle_rad, double no_go_angle_rad) {
	if (raycast(map, start->x, start->y, goal->x, goal->y)) {
		return { start, goal };
	}
	return {};
}