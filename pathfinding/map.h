#pragma once
#include <vector>
#include <memory>
#include "node.h"
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Search_traits_2.h>
#include <CGAL/Kd_tree.h>
#include <CGAL/Kd_tree_rectangle.h>
#include <CGAL/Orthogonal_k_neighbor_search.h>
#include <CGAL/Fuzzy_sphere.h>
#include <thread>

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_2 Point_2;
typedef CGAL::Search_traits_2<Kernel> Traits;
typedef CGAL::Kd_tree<Traits> Tree;
typedef CGAL::Orthogonal_k_neighbor_search<Traits> Neighbor_search;
typedef Neighbor_search::Tree Point_tree;
typedef CGAL::Fuzzy_sphere<Traits> Fuzzy_circle;
class Map {
public:
	//for initial construction
	Map(uint32_t map_width, uint32_t map_height);
	//for rotation
	Map(uint32_t size, std::shared_ptr<std::vector<float>> new_data, std::shared_ptr<std::vector<std::vector<Node>>> new_grid);
	~Map();
	Map* rotate(double map_angle_deg);
	void addNeighbors(int x, int y);
	Node* getNode(int x, int y);
	void generate_obstacles(int num_obstacles, int max_blob_size);
	bool isWalkable(float x, float y);
	bool isBlocked(float x, float y);
	int gridToIndex(float x, float y);
	Node* randomNode();
	void initPRM(float num_samples, float connection_radius_percent);
	std::vector<Node*> sampleNodes(int numNodes);
	void sampleGaussian(int numNodes, float target_x, float target_y, float std_dev);
	void addPRMNodes(std::vector<Node*> sampled_nodes);
	Node* addSinglePRMNode(float x, float y, float connection_radius);
	//std::vector<Node*> getNeighbors(Node* node);
	uint32_t width;
	uint32_t height;
	uint32_t max_dim;
	uint32_t half_height_diff;
	uint32_t half_width_diff;
	float prm_connection_radius = 0;
	//right now, we retain pointers to elements in this grid. This is not advisable, as if the vector is resized (as you insert more objects)
	//all of the pointers are invalidated. We manually call resize() to reserve as much space as we need in the constructor to work around this.
	//eventually, we should move to arrays instead.
	std::shared_ptr<std::vector<std::vector<Node>>> neighbors_grid;
	std::shared_ptr<std::vector<float>> data;
	std::shared_ptr<std::vector<Node*>> PRMNodes;
	std::unordered_map<PointKey, Node*> PRMNodeMap;
	Point_tree tree;
private:
	void create_blob(std::shared_ptr<std::vector<float>> grid, uint32_t blob_start_x, uint32_t blob_start_y, int blob_size);
	Node* findNodeByPosition(float x, float y);
};