#pragma once
//https://github.com/Ohohcakester/Any-Angle-Pathfinding/blob/master/src/algorithms/JumpPointSearch.java
#include "pathfinding_strategy_base.h"
#define TURN_WEIGHT 1

class JPSPathfindingStrategy : public PathfindingStrategyBase {
public:
	std::vector<std::shared_ptr<Node>> solve(Map& map, std::shared_ptr<Node> start, std::shared_ptr<Node> goal, double wind_angle_rad = 0, double no_go_angle_rad = 0);
private:
	float turn_penalty(std::shared_ptr<Node> previous, std::shared_ptr<Node> current, std::shared_ptr<Node> next);
	float heuristic(std::shared_ptr<Node> a, std::shared_ptr<Node> b);
	std::shared_ptr<Node> jumpU(Map& graph, int x, int y, int ex, int ey);
	std::shared_ptr<Node> jumpD(Map& graph, int x, int y, int ex, int ey);
	std::shared_ptr<Node> jumpR(Map& graph, int x, int y, int ex, int ey);
	std::shared_ptr<Node> jumpL(Map& graph, int x, int y, int ex, int ey);
	std::shared_ptr<Node> jumpUR(Map& graph, int x, int y, int ex, int ey);
	std::shared_ptr<Node> jumpUL(Map& graph, int x, int y, int ex, int ey);
	std::shared_ptr<Node> jumpDR(Map& graph, int x, int y, int ex, int ey);
	std::shared_ptr<Node> jumpDL(Map& graph, int x, int y, int ex, int ey);
	std::shared_ptr<Node> jump(Map& graph, int x, int y, int dx, int dy, int ex, int ey);
	std::vector<std::shared_ptr<Node>> find_jump_points(Map& graph, std::shared_ptr<Node> current, std::shared_ptr<Node> end);
	std::vector<std::pair<int, int>> jump_directions = { {0, 1}, {1, 0}, /*{0, -1},*/ {-1, 0}, {1, 1}, {1, -1}, {-1, -1}, {-1, 1}};

};