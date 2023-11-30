#pragma once
#include <vector>
class Map {
public:
	Map(uint32_t map_width, uint32_t map_height, int num_obstacles, int max_blob_size) {
		width = map_width;
		height = map_height;
		regenerate_map(num_obstacles, max_blob_size);
	}
	void regenerate_map(int num_obstacles, int max_blob_size);
	uint32_t width;
	uint32_t height;
	std::vector<float> data;
private:
	void create_blob(std::vector<float>& grid, int width, int blob_start_x, int blob_start_y, int blob_size);
};