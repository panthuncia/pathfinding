#pragma once
#include <vector>
class Map {
public:
	Map(uint32_t map_width, uint32_t map_height);
	Map rotate(double map_angle_deg);
	void generate_obstacles(int num_obstacles, int max_blob_size);
	uint32_t width;
	uint32_t height;
	std::vector<float> data;
private:
	void create_blob(std::vector<float>& grid, int width, int blob_start_x, int blob_start_y, int blob_size);
};