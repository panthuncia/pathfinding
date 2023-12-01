#pragma once
#include <vector>
#include "node.h"
class Map {
public:
	Map(uint32_t map_width, uint32_t map_height);
	Map rotate(double map_angle_deg);
	void addNeighbors(int x, int y);
	Node* getNode(int x, int y);
	void generate_obstacles(int num_obstacles, int max_blob_size);
	bool isWalkable(int x, int y);
	int gridToIndex(uint32_t x, uint32_t y);
	//std::vector<Node*> getNeighbors(Node* node);
	uint32_t width;
	uint32_t height;
	std::vector<std::vector<Node>> grid;
	std::vector<float> data;
private:
	void create_blob(std::vector<float>& grid, int width, int blob_start_x, int blob_start_y, int blob_size);
};