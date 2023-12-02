#pragma once
#include <vector>
#include <memory>
#include "node.h"
#include "../extern/annoy/src/annoylib.h"
#include "../extern/annoy/src/kissrandom.h"
class Map {
public:
	Map(uint32_t map_width, uint32_t map_height);
	~Map();
	void initPRM(int numSamples);
	Map rotate(double map_angle_deg);
	void addNeighbors(int x, int y);
	std::shared_ptr<Node> getNode(int x, int y);
	void generate_obstacles(int num_obstacles, int max_blob_size);
	bool isWalkable(int x, int y);
	bool isBlocked(int x, int y);
	int gridToIndex(uint32_t x, uint32_t y);
	std::vector<std::shared_ptr<Node>> sampleNodes(int numNodes);
	//std::vector<Node*> getNeighbors(Node* node);
	uint32_t width;
	uint32_t height;
	std::vector<std::vector<std::shared_ptr<Node>>> grid;
	std::vector<float> data;
	std::shared_ptr<Annoy::AnnoyIndex<int, int, Annoy::Euclidean, Annoy::Kiss32Random, Annoy::AnnoyIndexSingleThreadedBuildPolicy>> index;
	std::vector<std::shared_ptr<Node>> PRMNodes;
private:
	void create_blob(std::vector<float>& grid, int width, int blob_start_x, int blob_start_y, int blob_size);
};