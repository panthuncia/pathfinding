#include "prm_pathfinding_strategy.h"
#include <unordered_set>
#include "utilities.h"
float PRMPathfindingStrategy::heuristic(Node* a, Node* b) {
	return std::hypot(a->x - b->x, a->y - b->y);
}

float PRMPathfindingStrategy::turn_penalty(Node* previous, Node* current, Node* next) {
	// Calculate vectors from previous to current and from current to next
	float dx1 = current->x - previous->x;
	float dy1 = current->y - previous->y;
	float dx2 = next->x - current->x;
	float dy2 = next->y - current->y;

	// Calculate the dot product of the vectors
	float dotProduct = dx1 * dx2 + dy1 * dy2;

	// Calculate the magnitudes of the vectors
	float magnitude1 = sqrt(dx1 * dx1 + dy1 * dy1);
	float magnitude2 = sqrt(dx2 * dx2 + dy2 * dy2);

	// Calculate the cosine of the angle between the vectors
	float cosTheta = dotProduct / (magnitude1 * magnitude2);

	// Clamp the cosine value to the range [-1,1] to avoid any precision issues
	cosTheta = std::max(-1.0f, std::min(1.0f, cosTheta));

	// Calculate the actual angle in radians
	float angle = acos(cosTheta);

	// float angle_degrees = angle * 180.0 / M_PI;

	return angle * TURN_WEIGHT; // Scale the penalty by the angle
}

std::vector<Node*> PRMPathfindingStrategy::AStar(Map& map, Node* start, Node* goal, double wind_angle_rad, double no_go_angle_rad) {
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
			Node* originalCurrent = currentNode;
			while (currentNode != nullptr) {
				path.push_back(currentNode);
				currentNode = currentNode->parent;
			}
			std::reverse(path.begin(), path.end());
			originalCurrent->reset();
			for (Node* node : closedSet) {
				node->reset();
			}
			for (int i = 0; i < openSet.size(); i++) {
				Node* node = openSet.top();
				openSet.pop();
				node->reset();
			}
			return path;
		}

		closedSet.insert(currentNode);
		for (Node* neighbor : currentNode->neighbors) {
			if (closedSet.contains(neighbor) || !map.isWalkable(neighbor->x, neighbor->y) || is_in_nogo(currentNode, neighbor, wind_angle_rad, no_go_angle_rad)) {
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

	return std::vector<Node*>(); // Return an empty path if no path is found
}

std::vector<std::pair<double, double>> PRMPathfindingStrategy::solve(Map& map, Node* start, Node* goal, double wind_angle_rad, double no_go_angle_rad) {
	std::shared_ptr<Node> prmStart = map.addSinglePRMNode(start->x, start->y, map.prm_connection_radius);
	std::shared_ptr<Node> prmGoal = map.addSinglePRMNode(goal->x, goal->y, map.prm_connection_radius);
	auto path = AStar(map, prmStart.get(), prmGoal.get(), wind_angle_rad, no_go_angle_rad);
	return path_to_doubles(path);
}