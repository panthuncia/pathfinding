#pragma once
#include <vector>
#include <memory>
#include "node.h"
class Map {
public:
	//for initial construction
	Map(uint32_t map_width, uint32_t map_height);
	//for rotation
	Map(uint32_t size, std::shared_ptr<std::vector<float>> new_data, std::shared_ptr<std::vector<std::vector<Node>>> new_grid);
	Map rotate(double map_angle_deg);
	void addNeighbors(int x, int y);
	Node* getNode(int x, int y);
	void generate_obstacles(int num_obstacles, int max_blob_size);
	bool isWalkable(int x, int y);
	bool isBlocked(int x, int y);
	int gridToIndex(uint32_t x, uint32_t y);
	//std::vector<Node*> getNeighbors(Node* node);
	uint32_t width;
	uint32_t height;
	uint32_t max_dim;
	uint32_t half_height_diff;
	uint32_t half_width_diff;
	std::shared_ptr<std::vector<std::vector<Node>>> neighbors_grid;
	std::shared_ptr<std::vector<float>> data;
private:
	void create_blob(std::shared_ptr<std::vector<float>> grid, uint32_t blob_start_x, uint32_t blob_start_y, int blob_size);
};