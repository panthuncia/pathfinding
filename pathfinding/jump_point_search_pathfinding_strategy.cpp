#include "jump_point_search_pathfinding_strategy.h"
#include "utilities.h"
#include <unordered_set>
#include <float.h>
#include <iostream>
#include <chrono>
#include <string>
#include "utilities.h"

float JPSPathfindingStrategy::turn_penalty(Node* previous, Node* current, Node* next) {
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

float JPSPathfindingStrategy::heuristic(Node* a, Node* b) {
	return std::hypot(a->x - b->x, a->y - b->y);
}

Node* JPSPathfindingStrategy::jumpU(Map& graph, int x, int y, int ex, int ey) {
	while (true) {
		y += 1;
		if (graph.isBlocked(x, y - 1)) {
			if (graph.isBlocked(x - 1, y - 1)) {
				return nullptr;
			}
			else {
				if (!graph.isBlocked(x, y)) return graph.getNode(x, y);
			}
		}
		if (graph.isBlocked(x - 1, y - 1)) {
			if (!graph.isBlocked(x - 1, y)) return graph.getNode(x, y);
		}
		if (x == ex && y == ey) return graph.getNode(x, y);
	}
}

Node* JPSPathfindingStrategy::jumpD(Map& graph, int x, int y, int ex, int ey) {
	while (true) {
		y -= 1;
		if (graph.isBlocked(x, y)) {
			if (graph.isBlocked(x - 1, y)) {
				return nullptr;
			}
			else {
				if (!graph.isBlocked(x, y - 1)) return graph.getNode(x, y);
			}
		}
		if (graph.isBlocked(x - 1, y)) {
			if (!graph.isBlocked(x - 1, y - 1)) return graph.getNode(x, y);
		}
		if (x == ex && y == ey) return graph.getNode(x, y);
	}
}

Node* JPSPathfindingStrategy::jumpR(Map& graph, int x, int y, int ex, int ey) {
	while (true) {
		x += 1;
		if (graph.isBlocked(x - 1, y)) {
			if (graph.isBlocked(x - 1, y - 1)) {
				return nullptr;
			}
			else {
				if (!graph.isBlocked(x, y)) return graph.getNode(x, y);
			}
		}
		if (graph.isBlocked(x - 1, y - 1)) {
			if (!graph.isBlocked(x, y - 1)) return graph.getNode(x, y);
		}
		if (x == ex && y == ey) return graph.getNode(x, y);
	}
}

Node* JPSPathfindingStrategy::jumpL(Map& graph, int x, int y, int ex, int ey) {
	while (true) {
		x -= 1;
		if (graph.isBlocked(x, y)) {
			if (graph.isBlocked(x, y - 1)) {
				return nullptr;
			}
			else {
				if (!graph.isBlocked(x - 1, y)) return graph.getNode(x, y);
			}
		}
		if (graph.isBlocked(x, y - 1)) {
			if (!graph.isBlocked(x - 1, y - 1)) return graph.getNode(x, y);
		}
		if (x == ex && y == ey) return graph.getNode(x, y);
	}
}

Node* JPSPathfindingStrategy::jumpUR(Map& graph, int x, int y, int ex, int ey) {
	while (true) {
		x += 1;
		y += 1;
		if (graph.isBlocked(x - 1, y - 1)) return nullptr;
		if (x == ex && y == ey) return graph.getNode(x, y);
		// diagonal cannot be forced on vertices.
		if (jumpU(graph, x, y, ex, ey) != nullptr) return graph.getNode(x, y);
		if (jumpR(graph, x, y, ex, ey) != nullptr) return graph.getNode(x, y);
	}
}

Node* JPSPathfindingStrategy::jumpUL(Map& graph, int x, int y, int ex, int ey) {
	while (true) {
		x -= 1;
		y += 1;
		if (graph.isBlocked(x, y - 1)) return nullptr;
		if (x == ex && y == ey) return graph.getNode(x, y);
		// diagonal cannot be forced on vertices.
		if (jumpL(graph, x, y, ex, ey) != nullptr) return graph.getNode(x, y);
		if (jumpU(graph, x, y, ex, ey) != nullptr) return graph.getNode(x, y);
	}
}

Node* JPSPathfindingStrategy::jumpDR(Map& graph, int x, int y, int ex, int ey) {
	while (true) {
		x += 1;
		y -= 1;
		if (graph.isBlocked(x - 1, y)) return nullptr;
		if (x == ex && y == ey) return graph.getNode(x, y);
		// diagonal cannot be forced on vertices.
		if (jumpD(graph, x, y, ex, ey) != nullptr) return graph.getNode(x, y);
		if (jumpR(graph, x, y, ex, ey) != nullptr) return graph.getNode(x, y);
	}
}

Node* JPSPathfindingStrategy::jumpDL(Map& graph, int x, int y, int ex, int ey) {
	while (true) {
		x -= 1;
		y -= 1;
		if (graph.isBlocked(x, y)) return nullptr;
		if (x == ex && y == ey) return graph.getNode(x, y);
		// diagonal cannot be forced on vertices.
		if (jumpL(graph, x, y, ex, ey) != nullptr) return graph.getNode(x, y);
		if (jumpD(graph, x, y, ex, ey) != nullptr) return graph.getNode(x, y);
	}
}

Node* JPSPathfindingStrategy::jump(Map& graph, int x, int y, int dx, int dy, int ex, int ey) {
	if (dx < 0) {
		if (dy < 0) {
			return jumpDL(graph, x, y, ex, ey);
		}
		else if (dy > 0) {
			return jumpUL(graph, x, y, ex, ey);
		}
		else {
			return jumpL(graph, x, y, ex, ey);
		}
	}
	else if (dx > 0) {
		if (dy < 0) {
			return jumpDR(graph, x, y, ex, ey);
		}
		else if (dy > 0) {
			return jumpUR(graph, x, y, ex, ey);
		}
		else {
			return jumpR(graph, x, y, ex, ey);
		}
	}
	else {
		if (dy < 0) {
			return jumpD(graph, x, y, ex, ey);
		}
		else {
			return jumpU(graph, x, y, ex, ey);
		}
	}

}
std::vector<Node*> JPSPathfindingStrategy::find_jump_points(Map& graph,Node* current, Node* end) {
	std::vector<Node*> jump_points;
	for (auto p : jump_directions) {
		auto new_jump_point = jump(graph, current->x, current->y, p.first, p.second, end->x, end->y);
		if (new_jump_point != nullptr) {
			jump_points.push_back(new_jump_point);
		}
		//std::cout << "jump: " + std::to_string(new_jump_point->x) + ", " + std::to_string(new_jump_point->y) << std::endl;
	}
	return jump_points;
}

std::vector<std::pair<double, double>> JPSPathfindingStrategy::solve(Map& map, Node* start, Node* goal, double wind_angle_rad, double no_go_angle_rad) {
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

			return path_to_doubles(path);
		}

		closedSet.insert(currentNode);
		for (Node* neighbor : find_jump_points(map, currentNode, goal)) {
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

	return {}; // Return an empty path if no path is found
}