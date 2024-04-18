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

			//reset nodes
			while (!openSet.empty()) {
				auto node = openSet.top();
				openSet.pop();
				node->gCost = INFINITY;
			}
			for (auto node : closedSet) {
				node->gCost = INFINITY;
			}
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
	std::cout << "map angle deg:" + std::to_string(map_angle_deg);
	Map& rotated_map = map.rotate(map_angle_deg);

	auto transformed_start_doubles = rotateAndScale(start, map_angle_rad, map.max_dim, map.max_dim, rotated_map.max_dim, rotated_map.max_dim);
	std::pair<uint32_t, uint32_t> transformed_start_cell;
	if (transformed_start_doubles.first > map.max_dim-1) {
		transformed_start_cell.first = map.max_dim-1;
	}
	else if (transformed_start_doubles.first < 0) {
		transformed_start_cell.first = 0;
	}
	else {
		transformed_start_cell.first = uint32_t(transformed_start_doubles.first);
	}
	if (transformed_start_doubles.second > map.max_dim-1) {
		transformed_start_cell.second = map.max_dim-1;
	}
	else if (transformed_start_doubles.second < 0) {
		transformed_start_cell.second = 0;
	}
	else {
		transformed_start_cell.second = uint32_t(transformed_start_doubles.second);
	}
	auto transformed_goal_doubles = rotateAndScale(goal, map_angle_rad, map.max_dim, map.max_dim, rotated_map.max_dim, rotated_map.max_dim);
	std::pair<uint32_t, uint32_t> transformed_goal_cell;
	if (transformed_goal_doubles.first > map.max_dim-1) {
		transformed_goal_cell.first = map.max_dim-1;
	}
	else if (transformed_goal_doubles.first < 0) {
		transformed_goal_cell.first = 0;
	}
	else {
		transformed_goal_cell.first = uint32_t(transformed_goal_doubles.first);
	}
	if (transformed_goal_doubles.second > map.max_dim-1) {
		transformed_goal_cell.second = map.max_dim-1;
	}
	else if (transformed_goal_doubles.second < 0) {
		transformed_goal_cell.second = 0;
	}
	else {
		transformed_goal_cell.second = uint32_t(transformed_goal_doubles.second);
	}

	auto path = AStar(rotated_map, rotated_map.getNode(transformed_start_cell.first, transformed_start_cell.second), rotated_map.getNode(transformed_goal_cell.first, transformed_goal_cell.second));
	displayGrid(rotated_map.data, rotated_map.max_dim, rotated_map.max_dim, path_to_doubles(path), 90, "rotated grid");
	return rotate_path_doubles(path, map.max_dim, map.max_dim, map.max_dim, map.max_dim, map_angle_deg);
}