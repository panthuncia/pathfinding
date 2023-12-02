#pragma once
#include "pathfinding_strategy_base.h"
#define TURN_WEIGHT 1

class AStarPathfindingStrategy : public PathfindingStrategyBase {
public:
	std::vector<Node*> solve(Map& map, Node* start, Node* goal, double wind_angle_rad = 0, double no_go_angle_rad = 0);
private:
	float turn_penalty(Node* previous, Node* current, Node* next);
	float heuristic(Node* a, Node* b);
};