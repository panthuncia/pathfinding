#include "astar_pathfinding_strategy.h"
#include "utilities.h"
#include <unordered_set>
#include <float.h>
#include <iostream>
#include <chrono>
#include <string>

float AStarPathfindingStrategy::turn_penalty(std::shared_ptr<Node> previous, std::shared_ptr<Node> current, std::shared_ptr<Node> next) {
	if (current->x - previous->x != 0 && next->x - current->x != 0) {
		double slope1 = (current->y - previous->y) / (current->x - previous->x);
		double slope2 = (next->y - current->y) / (next->x - current->x);
		if (double_equals(slope1, slope2)) {
			return 0;
		}
		else {
			return TURN_WEIGHT;
		}
	}
	else {
		if (previous->x == current->x == next->x || previous->y == current->y == next->y) {
			return 0;
		}
		else {
			return TURN_WEIGHT;
		}
	}
}

float AStarPathfindingStrategy::heuristic(std::shared_ptr<Node> a, std::shared_ptr<Node> b) {
	return std::hypot(a->x - b->x, a->y - b->y);
}

std::vector<std::shared_ptr<Node>> AStarPathfindingStrategy::solve(Map& map, std::shared_ptr<Node> start, std::shared_ptr<Node> goal, double wind_angle_rad, double no_go_angle_rad) {
	std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>, CompareNode> openSet;
	std::unordered_set<std::shared_ptr<Node>> closedSet;
	start->gCost = 0;
	start->hCost = heuristic(start, goal);
	start->calculateFCost();
	openSet.push(start);

	while (!openSet.empty()) {
		std::shared_ptr<Node> currentNode = openSet.top();
		openSet.pop();
		if (currentNode == goal) {
			std::vector<std::shared_ptr<Node>> path;
			while (currentNode != nullptr) {
				path.push_back(currentNode);
				currentNode = currentNode->parent;
			}
			std::reverse(path.begin(), path.end());

			return path;
		}

		closedSet.insert(currentNode);
		for (std::shared_ptr<Node> neighbor : currentNode->neighbors) {
			if (closedSet.contains(neighbor) || !map.isWalkable(neighbor->x, neighbor->y)) {
				continue;
			}
			float currentTurnPenalty = 0;
			if (currentNode->parent != nullptr) {
				currentTurnPenalty = turn_penalty(currentNode->parent, currentNode, neighbor);
			}
			float tentativeGCost = currentNode->gCost + heuristic(currentNode, neighbor)+currentTurnPenalty;
			if (tentativeGCost < neighbor->gCost) {
				neighbor->parent = currentNode;
				neighbor->gCost = tentativeGCost;
				neighbor->hCost = heuristic(neighbor, goal);
				neighbor->calculateFCost();
				openSet.push(neighbor);
			}
		}
	}

	return std::vector<std::shared_ptr<Node>>(); // Return an empty path if no path is found
}