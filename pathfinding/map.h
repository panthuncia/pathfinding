#pragma once
#include <vector>
#include <memory>
#include "node.h"
#include "../extern/annoy/src/annoylib.h"
#include "../extern/annoy/src/kissrandom.h"
class Map {
public:
	//for initial construction
	Map(uint32_t map_width, uint32_t map_height);
	//for rotation
	Map(uint32_t size, std::shared_ptr<std::vector<float>> new_data, std::shared_ptr<std::vector<std::vector<Node>>> new_grid, std::shared_ptr<std::vector<std::shared_ptr<Node>>> new_prm_nodes);
	Map rotate(double map_angle_deg);
	void addNeighbors(int x, int y);
	Node* getNode(int x, int y);
	void generate_obstacles(int num_obstacles, int max_blob_size);
	bool isWalkable(int x, int y);
	bool isBlocked(int x, int y);
	int gridToIndex(uint32_t x, uint32_t y);
	Node* randomNode();
	void initPRM(int numSamples);
	std::vector<std::shared_ptr<Node>> sampleNodes(int numNodes);
	std::shared_ptr<Node> addSinglePRMNode(uint32_t x, uint32_t y, uint32_t num_neighbors);
	//std::vector<Node*> getNeighbors(Node* node);
	uint32_t width;
	uint32_t height;
	uint32_t max_dim;
	uint32_t half_height_diff;
	uint32_t half_width_diff;
	//right now, we retain pointers to elements in this grid. This is not advisable, as if the vector is resized (as you insert more objects)
	//all of the pointers are invalidated. We manually call resize() to reserve as much space as we need in the constructor to work around this.
	//eventually, we should move to arrays instead.
	std::shared_ptr<std::vector<std::vector<Node>>> neighbors_grid;
	std::shared_ptr<std::vector<float>> data;
	std::shared_ptr<std::vector<std::shared_ptr<Node>>> PRMNodes;
	std::shared_ptr<Annoy::AnnoyIndex<int, int, Annoy::Euclidean, Annoy::Kiss32Random, Annoy::AnnoyIndexSingleThreadedBuildPolicy>> index;
private:
	void create_blob(std::shared_ptr<std::vector<float>> grid, uint32_t blob_start_x, uint32_t blob_start_y, int blob_size);
};