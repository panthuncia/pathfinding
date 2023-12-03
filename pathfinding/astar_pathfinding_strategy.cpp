#include "astar_pathfinding_strategy.h"
#include "utilities.h"
#include <unordered_set>
#include <float.h>
#include <iostream>
#include <chrono>
#include <string>
#define _USE_MATH_DEFINES
#include <math.h>
float AStarPathfindingStrategy::turn_penalty(Node* previous, Node* current, Node* next) {
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

float AStarPathfindingStrategy::heuristic(Node* a, Node* b) {
	return std::hypot(a->x - b->x, a->y - b->y);
}

std::vector<Node*> AStarPathfindingStrategy::AStar(Map& map, Node* start, Node* goal) {
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
			float currentTurnPenalty = 0;
			if (currentNode->parent != nullptr) {
				currentTurnPenalty = turn_penalty(currentNode->parent, currentNode, neighbor);
			}
			float tentativeGCost = currentNode->gCost + heuristic(currentNode, neighbor) + currentTurnPenalty;
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

std::vector<std::pair<double, double>> AStarPathfindingStrategy::solve(Map& map, Node* start, Node* goal, double wind_angle_rad, double no_go_angle_rad) {
	//rotate map to enable wind restriction
	double map_angle_rad = wind_angle_rad - M_PI / 2;
	double map_angle_deg = map_angle_rad * (180 / M_PI);
	Map rotated_map = map.rotate(map_angle_deg);

	auto transformed_start_doubles = rotateAndScale(start, map_angle_rad, map.max_dim, map.max_dim, rotated_map.max_dim, rotated_map.max_dim);
	auto transformed_goal_doubles = rotateAndScale(goal, map_angle_rad, map.max_dim, map.max_dim, rotated_map.max_dim, rotated_map.max_dim);

	auto path = AStar(rotated_map, rotated_map.getNode(transformed_start_doubles.first, transformed_start_doubles.second), rotated_map.getNode(transformed_goal_doubles.first, transformed_goal_doubles.second));
	return rotate_path_doubles(path, map.max_dim, map.max_dim, map.max_dim, map.max_dim, map_angle_deg);
}