#include "astar_pathfinding_strategy.h"
#include "utilities.h"
#include <unordered_set>
#include <float.h>
#include <iostream>

std::unordered_map<Node, double, NodeHasher>* pFScoreMap;

int gridToIndex(Map map, uint32_t x, uint32_t y) {
	return y * map.width + x;
}
bool isCellWalkable(Map map, uint32_t x, uint32_t y) {
	if (x >= 0 and y >= 0 and x < map.width and y < map.height and map.data[gridToIndex(map, x, y)] <0.5)
		return true;
	return false;
}
std::vector<Node> neighbors_of_7(Map map, uint32_t x, uint32_t y){
	std::vector<Node> neighbors;
	if (isCellWalkable(map, x, y - 1))
		neighbors.push_back(Node(x, y - 1));
	if (isCellWalkable(map, x - 1, y))
		neighbors.push_back(Node(x - 1, y));
	if (isCellWalkable(map, x + 1, y))
		neighbors.push_back(Node(x + 1, y));
	/*if (isCellWalkable(map, x, y + 1))
		neighbors.push_back(Node(x, y + 1));*/
	if (isCellWalkable(map, x - 1, y - 1))
		neighbors.push_back(Node(x - 1, y - 1));
	if (isCellWalkable(map, x + 1, y - 1))
		neighbors.push_back(Node(x + 1, y - 1));
	if (isCellWalkable(map, x - 1, y + 1))
		neighbors.push_back(Node(x - 1, y + 1));
	if (isCellWalkable(map, x + 1, y + 1))
		neighbors.push_back(Node(x + 1, y + 1));
	return neighbors;
}


std::vector<Node> reconstructPath(Node end, NodeUnorderedMap cameFrom) {
	std::vector<Node> path;
	path.push_back(end);
	while (cameFrom.find(end) != cameFrom.end()) {
		end = cameFrom[end];
		path.insert(path.begin(), end);
	}
	return path;
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
std::vector<Node> AStarPathfindingStrategy::solve(Map map, Node start, Node goal, double wind_angle_rad, double no_go_angle) {
	NodeUnorderedMap cameFrom;
	std::unordered_map<Node, double, NodeHasher> gScoreMap;
	std::unordered_map<Node, double, NodeHasher> fScoreMap;
	//This is probably unsafe, but I'm not sure how else to access fScoreMap in the node comparator
	pFScoreMap = &fScoreMap;
	NodePriorityQueue openSet;
	//std::unordered_set<Node> open;
	std::unordered_set<Node, NodeHasher> closed;
	Node current = start;
	gScoreMap[start] = 0;
	double startF = distance(start.x, start.y, goal.x, goal.y);
	fScoreMap[start] = startF;
	if (current == goal) {
		return reconstructPath(goal, cameFrom);
	}
	else {
		fScoreMap[current] = startF;
		openSet.push(current);
		//open.insert(current);
;	}
	while (current != goal && !openSet.size() == 0) {
		current = openSet.top();
		openSet.pop();
		//open.erase(current);
		std::vector<Node> neighbors = neighbors_of_7(map, current.x, current.y);
		for (Node neighbor : neighbors) {
			if (closed.find(neighbor) != closed.end()) {
				continue;
			}
			float currentTurnPenalty = 0;
			if (cameFrom.find(current) != cameFrom.end()) {
				currentTurnPenalty = turn_penalty(cameFrom[current], current, neighbor);
			}
			double tentative_gscore = gScoreMap[current] + distance(neighbor.x, neighbor.y, current.x, current.y) + currentTurnPenalty;
			if (gScoreMap.find(neighbor) == gScoreMap.end()) {
				gScoreMap[neighbor] = DBL_MAX;
			}
			if (tentative_gscore < gScoreMap[neighbor]) {
				cameFrom[neighbor] = current;
				gScoreMap[neighbor] = tentative_gscore;
				double f = tentative_gscore + distance(neighbor.x, neighbor.y, goal.x, goal.y);
				fScoreMap[neighbor] = f;
				//if (open.find(neighbor) != open.end()) {
				//	//do we need to do this?
				//}
				openSet.push(neighbor);
				closed.insert(neighbor);
			}
		}
	}
	if (openSet.size() == 0) {
		std::cout << "No path found!" << std::endl;
		std::vector<Node> empty;
		return empty;
	}
	return reconstructPath(goal, cameFrom);
}