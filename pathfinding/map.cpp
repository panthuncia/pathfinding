#include "map.h"
#include <vector>
#include <random>
#include <opencv2/opencv.hpp>
Map::Map(uint32_t map_width, uint32_t map_height) {
    height = map_height;
    width = map_width;
    grid.resize(height, std::vector<Node>(width));

    // Initialize nodes and their neighbors
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            grid[y][x] = Node(x, y);
            addNeighbors(x, y);
        }
    }
}
void Map::addNeighbors(int x, int y) {
    std::vector<std::pair<int, int>> neighborOffsets = { {1, 0}, {0, 1}, {-1, 0}, /*{0, -1},*/ {1, 1}, {1, -1}, {-1, -1}, {-1, 1}}; // 8-directional

    for (auto& offset : neighborOffsets) {
        int nx = x + offset.first;
        int ny = y + offset.second;

        if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
            grid[y][x].neighbors.push_back(&grid[ny][nx]);
        }
    }
}

Node* Map::getNode(int x, int y) {
    return &grid[y][x];
}

void Map::generate_obstacles(int num_obstacles, int max_blob_size) {
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

Map Map::rotate(double map_angle_deg) {
    //rotate map
    cv::Mat grid = cv::Mat(height, width, CV_32FC1, data.data());
    cv::Point2f center((grid.cols - 1) / 2.0, (grid.rows - 1) / 2.0);    cv::Mat rot = cv::getRotationMatrix2D(center, map_angle_deg, 1.0);
    cv::Rect2f bbox = cv::RotatedRect(cv::Point2f(), grid.size(), map_angle_deg).boundingRect2f();
    rot.at<double>(0, 2) += bbox.width / 2.0 - grid.cols / 2.0;
    rot.at<double>(1, 2) += bbox.height / 2.0 - grid.rows / 2.0;

    cv::Mat rotated_grid;
    cv::warpAffine(grid, rotated_grid, rot, bbox.size(), cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(1.0));
    int newHeight = rotated_grid.rows;
    int newWidth = rotated_grid.cols;
    // Ensure the data type is correct
    if (rotated_grid.type() != CV_32FC1) {
        rotated_grid.convertTo(rotated_grid, CV_32FC1);
    }

    // Flatten the matrix if it's not already a single row or single column
    if (rotated_grid.rows > 1 && rotated_grid.cols > 1) {
        rotated_grid = rotated_grid.reshape(1, 1); // Reshape to a single row
    }

    // Convert cv::Mat to std::vector<float>
    std::vector<float> rotated_vector;
    rotated_vector.assign((float*)rotated_grid.datastart, (float*)rotated_grid.dataend);
    Map new_map = Map(newWidth, newHeight);
    new_map.data = rotated_vector;
    return new_map;
}

bool Map::isWalkable(int x, int y) {
    if (x >= 0 and y >= 0 and x < width and y < height and data[gridToIndex(x, y)] < 0.5)
        return true;
    return false;
}

bool Map::isBlocked(int x, int y) {
    if (x >= 0 and y >= 0 and x < width and y < height and data[gridToIndex(x, y)] < 0.5)
        return false;
    return true;
}
int Map::gridToIndex(uint32_t x, uint32_t y) {
    return y * width + x;
}