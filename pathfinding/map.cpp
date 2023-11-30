#include "map.h"
#include <vector>
#include <random>
void Map::regenerate_map(int num_obstacles, int max_blob_size) {
    data = std::vector<float>(width * height, 0.0f); // Initialize grid with zeros

    std::random_device rd; // Obtain a random number from hardware
    std::mt19937 eng(rd()); // Seed the generator
    std::uniform_int_distribution<> distrX(0, width - 1); // Define range for width
    std::uniform_int_distribution<> distrY(0, height - 1); // Define range for height
    std::uniform_int_distribution<> distrSize(1, max_blob_size); // Define range for blob size

    for (int i = 0; i < num_obstacles; ++i) {
        int blob_start_x = distrX(eng);
        int blob_start_y = distrY(eng);
        int blob_size = distrSize(eng);

        create_blob(data, width, blob_start_x, blob_start_y, blob_size);
    }

}
void Map::create_blob(std::vector<float>& grid, int width, int blob_start_x, int blob_start_y, int blob_size) {
    std::vector<std::pair<int, int>> directions = { {1, 0}, {0, 1}, {-1, 0}, {0, -1} };

    std::random_device rd;
    std::mt19937 eng(rd());
    std::uniform_int_distribution<> distr(0, directions.size() - 1);

    for (int i = 0; i < blob_size; ++i) {
        int index = blob_start_y * width + blob_start_x;
        grid[index] = 1.0f; // Set the current position to 1

        // Randomly choose a direction
        auto [dx, dy] = directions[distr(eng)];

        // Update start_x and start_y, ensuring they stay within bounds
        blob_start_x = std::max(0, std::min(width - 1, blob_start_x + dx));
        blob_start_y = std::max(0, std::min(static_cast<int>(grid.size() / width) - 1, blob_start_y + dy));
    }
}