#pragma once
#include "pathfinding_strategy_base.h"
#define TURN_WEIGHT 1

class AStarPathfindingStrategy : public PathfindingStrategyBase {
public:
	std::vector<std::shared_ptr<Node>> solve(Map& map, std::shared_ptr<Node> start, std::shared_ptr<Node> goal, double wind_angle_rad = 0, double no_go_angle_rad = 0);
private:
	float turn_penalty(std::shared_ptr<Node> previous, std::shared_ptr<Node> current, std::shared_ptr<Node> next);
	float heuristic(std::shared_ptr<Node> a, std::shared_ptr<Node> b);
};