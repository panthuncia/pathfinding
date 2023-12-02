#pragma once
//https://github.com/Ohohcakester/Any-Angle-Pathfinding/blob/master/src/algorithms/JumpPointSearch.java
#include "pathfinding_strategy_base.h"
#define TURN_WEIGHT 1

class JPSPathfindingStrategy : public PathfindingStrategyBase {
public:
	std::vector<Node*> solve(Map& map, Node* start, Node* goal, double wind_angle_rad = 0, double no_go_angle_rad = 0);
private:
	float turn_penalty(Node* previous, Node* current, Node* next);
	float heuristic(Node* a, Node* b);
	Node* jumpU(Map& graph, int x, int y, int ex, int ey);
	Node* jumpD(Map& graph, int x, int y, int ex, int ey);
	Node* jumpR(Map& graph, int x, int y, int ex, int ey);
	Node* jumpL(Map& graph, int x, int y, int ex, int ey);
	Node* jumpUR(Map& graph, int x, int y, int ex, int ey);
	Node* jumpUL(Map& graph, int x, int y, int ex, int ey);
	Node* jumpDR(Map& graph, int x, int y, int ex, int ey);
	Node* jumpDL(Map& graph, int x, int y, int ex, int ey);
	Node* jump(Map& graph, int x, int y, int dx, int dy, int ex, int ey);
	std::vector<Node*> find_jump_points(Map& graph, Node* current, Node* end);
	std::vector<std::pair<int, int>> jump_directions = { {0, 1}, {1, 0}, /*{0, -1},*/ {-1, 0}, {1, 1}, {1, -1}, {-1, -1}, {-1, 1}};

};