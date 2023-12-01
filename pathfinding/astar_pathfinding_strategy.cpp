#include "astar_pathfinding_strategy.h"
#include "utilities.h"
#include <unordered_set>
#include <float.h>
#include <iostream>
#include <chrono>
#include <string>

int gridToIndex(Map map, uint32_t x, uint32_t y) {
	return y * map.width + x;
}
bool isCellWalkable(Map map, uint32_t x, uint32_t y) {
	if (x >= 0 and y >= 0 and x < map.width and y < map.height and map.data[gridToIndex(map, x, y)] <0.5)
		return true;
	return false;
}

float turn_penalty(Node previous, Node current, Node next) {
	if (current.x - previous.x != 0 && next.x - current.x != 0) {
		double slope1 = (current.y - previous.y) / (current.x - previous.x);
		double slope2 = (next.y - current.y) / (next.x - current.x);
		if (double_equals(slope1, slope2)) {
			return 0;
		}
		else {
			return TURN_WEIGHT;
		}
	}
	else {
		if (previous.x == current.x == next.x || previous.y == current.y == next.y) {
			return 0;
		}
		else {
			return TURN_WEIGHT;
		}
	}
}

float heuristic(Node* a, Node* b) {
	return std::hypot(a->x - b->x, a->y - b->y);
}

std::vector<Node*> AStarPathfindingStrategy::solve(Map& map, Node* start, Node* goal, double wind_angle_rad, double no_go_angle) {
	std::priority_queue<Node*, std::vector<Node*>, CompareNode> openSet;
	std::unordered_set<Node*> closedSet;
	start->gCost = 0;
	start->hCost = heuristic(start, goal);
	start->calculateFCost();
	openSet.push(start);

	while (!openSet.empty()) {
		Node* currentNode = openSet.top();
		openSet.pop();

		if (currentNode == goal) {
			std::vector<Node*> path;
			while (currentNode != nullptr) {
				path.push_back(currentNode);
				currentNode = currentNode->parent;
			}
			std::reverse(path.begin(), path.end());
			return path;
		}

		closedSet.insert(currentNode);

		for (Node* neighbor : currentNode->neighbors) {
			if (closedSet.contains(neighbor) || !map.isWalkable(neighbor->x, neighbor->y)) {
				continue;
			}

			float tentativeGCost = currentNode->gCost + heuristic(currentNode, neighbor);
			if (tentativeGCost < neighbor->gCost) {
				neighbor->parent = currentNode;
				neighbor->gCost = tentativeGCost;
				neighbor->hCost = heuristic(neighbor, goal);
				neighbor->calculateFCost();
				openSet.push(neighbor);
			}
		}
	}

	return std::vector<Node*>(); // Return an empty path if no path is found
}