#pragma once
#include "pathfinding_strategy_base.h"
class PRMPathfindingStrategy : public PathfindingStrategyBase {
public:
	std::vector<std::pair<double, double>> solve(Map& map, Node* start, Node* goal, double wind_angle_rad = 0, double no_go_angle_rad = 0);
private:
	std::vector<Node*> AStar(Map& map, Node* start, Node* goal);
	float heuristic(Node* a, Node* b);
};