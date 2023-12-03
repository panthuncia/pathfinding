#include "prm_pathfinding_strategy.h"
#include <unordered_set>
#include "utilities.h"
float PRMPathfindingStrategy::heuristic(Node* a, Node* b) {
	return std::hypot(a->x - b->x, a->y - b->y);
}

std::vector<Node*> PRMPathfindingStrategy::AStar(Map& map, Node* start, Node* goal) {
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
			//float currentTurnPenalty = 0;
			//if (currentNode->parent != nullptr) {
			//	currentTurnPenalty = turn_penalty(currentNode->parent, currentNode, neighbor);
			//}
			float tentativeGCost = currentNode->gCost + heuristic(currentNode, neighbor);// +currentTurnPenalty;
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

std::vector<std::pair<double, double>> PRMPathfindingStrategy::solve(Map& map, Node* start, Node* goal, double wind_angle_rad, double no_go_angle_rad) {
	auto prmStart = map.addSinglePRMNode(start->x, start->y, 10);
	auto prmGoal = map.addSinglePRMNode(goal->x, goal->y, 10);
	auto path = AStar(map, prmStart.get(), prmGoal.get());
	return path_to_doubles(path);
}