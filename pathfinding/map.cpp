#include "map.h"
#include <vector>
#include <random>
#include <chrono>
#include <cmath>
#include <opencv2/opencv.hpp>
#include "raycast.h"
Map::Map(uint32_t map_width, uint32_t map_height) {

    height = map_height;
    width = map_width;

    //calculate maximum integer dimension
    max_dim = ceil(sqrt(pow(height, 2) + pow(width, 2)));

    data = std::make_shared<std::vector<float>>(max_dim * max_dim, 1.0f); // Initialize grid with ones

    //fill interior area with 0
    half_height_diff = (max_dim - height) / 2;
    half_width_diff = (max_dim - width) / 2;

    for (uint32_t y = half_height_diff; y < height + half_height_diff; y++)
        for (uint32_t x = half_width_diff; x < width + half_width_diff; x++)
            data->at(y * max_dim + x) = 0.0;

    neighbors_grid = std::make_shared<std::vector<std::vector<Node>>>();
    neighbors_grid->resize(max_dim, std::vector<Node>(max_dim));

    // Initialize nodes and their neighbors
    for (int y = 0; y < max_dim; ++y) {
        for (int x = 0; x < max_dim; ++x) {
            neighbors_grid->at(y).at(x) = Node(x, y);
            addNeighbors(x, y);
        }
    }

    PRMNodes = std::make_shared<std::vector<std::shared_ptr<Node>>>();
}

Map::Map(uint32_t size, std::shared_ptr<std::vector<float>> new_data, std::shared_ptr<std::vector<std::vector<Node>>> new_grid, std::shared_ptr<std::vector<std::shared_ptr<Node>>> new_prm_nodes) {
    max_dim = size;
    data = new_data;
    neighbors_grid = new_grid;
    PRMNodes = new_prm_nodes;
}


void Map::addNeighbors(int x, int y) {
    std::vector<std::pair<int, int>> neighborOffsets = { {1, 0}, {0, 1}, {-1, 0}, /*{0, -1},*/ {1, 1}, {1, -1}, {-1, -1}, {-1, 1}}; // 8-directional

    for (auto& offset : neighborOffsets) {
        int nx = x + offset.first;
        int ny = y + offset.second;

        if (nx >= 0 && nx < max_dim && ny >= 0 && ny < max_dim) {
            neighbors_grid->at(y).at(x).neighbors.push_back(&neighbors_grid->at(ny).at(nx));
        }
    }
}

Node* Map::getNode(int x, int y) {
    return &neighbors_grid->at(y).at(x);
}

std::vector<std::shared_ptr<Node>> Map::sampleNodes(int numNodes) {
    std::vector<std::shared_ptr<Node>> nodes;
    for (int i = 0; i < numNodes; ++i) {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<float> x_dis(half_width_diff, max_dim-half_width_diff);
        std::uniform_real_distribution<float> y_dis(half_height_diff, max_dim - half_height_diff);
        float rand_x = x_dis(gen);
        float rand_y = y_dis(gen);
        std::shared_ptr<Node> node = std::make_shared<Node>(rand_x, rand_y);
        nodes.push_back(node);
    }
    return nodes;
}

std::vector<std::shared_ptr<Node>> Map::sampleGaussian(int numNodes, float target_x, float target_y, float std_dev) {
    std::vector<std::shared_ptr<Node>> nodes;
    std::random_device rd;  
    std::mt19937 gen(rd());
    std::normal_distribution<> d_x(target_x, std_dev);
    std::normal_distribution<> d_y(target_y, std_dev);

    for (int i = 0; i < numNodes; ++i) {
        float rand_x = d_x(gen);
        float rand_y = d_y(gen);

        // Ensure that the sampled points are within bounds
        rand_x = std::max(0.0f, std::min(rand_x, float(max_dim)));
        rand_y = std::max(0.0f, std::min(rand_y, float(max_dim)));

        std::shared_ptr<Node> node = std::make_shared<Node>(rand_x, rand_y);
        nodes.push_back(node);
    }
    return nodes;
}

void Map::initPRM(float samples_per_unit_squared, float connection_radius) {

    // Start with a low-resolution grid
    int step = 5;
    float hypot = sqrt(2 * float(pow(step, 2)));
    for (int i = half_height_diff; i < max_dim - half_height_diff; i += step) {
        for (int j = half_width_diff; j < max_dim - half_width_diff; j += step) {
            addSinglePRMNode(i, j, hypot+0.1);
        }
    }

    prm_connection_radius = connection_radius;
    int num_samples = samples_per_unit_squared * ((max_dim - 2 * half_height_diff) * (max_dim - 2 * half_width_diff));
    auto sampled_nodes = sampleNodes(num_samples);

    //PRMNodes->insert(PRMNodes->begin(), sampled_nodes.begin(), sampled_nodes.end());
    int i = 0;
    // Insert points into the tree
    for (auto& node : sampled_nodes) {
        PointKey key{ node->x, node->y };
        PRMNodeMap[key] = node;
        Point_2 point(node->x, node->y);
        tree.insert(point);
    }

    // Now, find the nearest neighbors for each point
    int k = 10; // Number of nearest neighbors to find
    //for (auto& node : sampled_nodes) {
    //    Point_2 query(node->x, node->y);
    //    Neighbor_search search(tree, query, k);

    //    for (auto it = search.begin(); it != search.end(); ++it) {
    //        Point_2 found = it->first;
    //        // Convert found Point_2 back to node indices or references
    //        auto neighborNode = findNodeByPosition(found.x(), found.y());
    //        PRMNodes->push_back(neighborNode);
    //        if (neighborNode != node) {
    //            if (raycast(*this, node->x, node->y, neighborNode->x, neighborNode->y)) {
    //                // Connect nodes
    //                node->neighbors.push_back(neighborNode.get());
    //                neighborNode->neighbors.push_back(node.get());
    //            }
    //        }
    //    }
    //}
    for (auto& node : sampled_nodes) {
        Point_2 query(node->x, node->y);
        Fuzzy_circle region(query, connection_radius, 0.0 /*exact search*/);
        std::vector<Point_2> result;
        tree.search(std::back_inserter(result), region);
        for (const auto& found : result) {
            // Convert found Point_2 back to node indices or references
            auto neighborNode = findNodeByPosition(found.x(), found.y());
            PRMNodes->push_back(neighborNode);
            if (neighborNode != node) {
                if (raycast(*this, node->x, node->y, neighborNode->x, neighborNode->y)) {
                    // Connect nodes
                    node->neighbors.push_back(neighborNode.get());
                    neighborNode->neighbors.push_back(node.get());
                }
            }
        }
    }
}

std::shared_ptr<Node> Map::findNodeByPosition(float x, float y) {
    PointKey key{ x, y };
    auto it = PRMNodeMap.find(key);
    if (it != PRMNodeMap.end()) {
        return it->second;
    }
    return nullptr; // If no node found
}

std::shared_ptr<Node> Map::addSinglePRMNode(float x, float y, float connection_radius) {
    int id = PRMNodes->size();
    auto new_node = std::make_shared<Node>(x, y);
    PointKey key{ new_node->x, new_node->y };
    PRMNodeMap[key] = new_node;
    Point_2 point(new_node->x, new_node->y);
    tree.insert(point);

    Fuzzy_circle region(point, connection_radius, 0.0 /*exact search*/);
    std::vector<Point_2> result;
    tree.search(std::back_inserter(result), region);

    for (const auto& found : result) {
        // Convert found Point_2 back to node indices or references
        auto neighborNode = findNodeByPosition(found.x(), found.y());
        PRMNodes->push_back(neighborNode);
        if (neighborNode != new_node) {
            if (raycast(*this, new_node->x, new_node->y, neighborNode->x, neighborNode->y)) {
                // Connect nodes
                new_node->neighbors.push_back(neighborNode.get());
                neighborNode->neighbors.push_back(new_node.get());
            }
        }
    }
    PRMNodes->push_back(new_node);
    return(new_node);
}

Node* Map::randomNode() {
    std::random_device rd;
    std::mt19937 eng(rd());
    std::uniform_int_distribution<> distrX(0, width - 1);
    std::uniform_int_distribution<> distrY(0, height - 1);

    return getNode(distrX(eng)+half_width_diff, distrY(eng)+half_height_diff);
}

void Map::generate_obstacles(int num_obstacles, int max_blob_size) {
    std::random_device rd; // Obtain a random number from hardware
    std::mt19937 eng(rd()); // Seed the generator
    std::uniform_int_distribution<> distrX(0, width - 1); // Define range for width
    std::uniform_int_distribution<> distrY(0, height - 1); // Define range for height
    std::uniform_int_distribution<> distrSize(1, max_blob_size); // Define range for blob size

    for (int i = 0; i < num_obstacles; ++i) {
        int blob_start_x = distrX(eng)+half_width_diff;
        int blob_start_y = distrY(eng)+half_height_diff;
        int blob_size = distrSize(eng);

        create_blob(data, blob_start_x, blob_start_y, blob_size);
    }

}
void Map::create_blob(std::shared_ptr<std::vector<float>> grid, uint32_t blob_start_x, uint32_t blob_start_y, int blob_size) {
    std::vector<std::pair<int, int>> directions = { {1, 0}, {0, 1}, {-1, 0}, {0, -1} };

    std::random_device rd;
    std::mt19937 eng(rd());
    std::uniform_int_distribution<> distr(0, directions.size() - 1);

    for (uint32_t i = 0; i < blob_size; ++i) {
        uint32_t index = blob_start_y * max_dim + blob_start_x;
        grid->at(index) = 1.0f; // Set the current position to 1

        // Randomly choose a direction
        auto [dx, dy] = directions[distr(eng)];

        // Update start_x and start_y, ensuring they stay within bounds
        blob_start_x = std::max(uint32_t(0), std::min(max_dim - 1, blob_start_x + dx));
        blob_start_y = std::max(uint32_t(0), std::min(static_cast<uint32_t>(grid->size() / max_dim) - 1, blob_start_y + dy));
    }
}

Map* Map::rotate(double map_angle_deg) {
    //rotate map
    cv::Mat mat = cv::Mat(max_dim, max_dim, CV_32FC1, data->data());
    cv::Point2f center((mat.cols - 1) / 2.0, (mat.rows - 1) / 2.0);    cv::Mat rot = cv::getRotationMatrix2D(center, map_angle_deg, 1.0);

    cv::Mat rotated_mat;
    cv::warpAffine(mat, rotated_mat, rot, mat.size(), cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(1.0));

    // Ensure the data type is correct
    if (rotated_mat.type() != CV_32FC1) {
        rotated_mat.convertTo(rotated_mat, CV_32FC1);
    }

    // Flatten the matrix if it's not already a single row or single column
    if (rotated_mat.rows > 1 && rotated_mat.cols > 1) {
        rotated_mat = rotated_mat.reshape(1, 1); // Reshape to a single row
    }

    // Convert cv::Mat to std::vector<float>
    auto rotated_vector = std::make_shared<std::vector<float>>();
    rotated_vector->assign((float*)rotated_mat.datastart, (float*)rotated_mat.dataend);
    return new Map(max_dim, rotated_vector, neighbors_grid, PRMNodes);
}

bool Map::isWalkable(float x, float y) {
    if (x >= 0 and y >= 0 and x < max_dim and y < max_dim and data->at(gridToIndex(x, y)) < 0.5)
        return true;
    return false;
}

bool Map::isBlocked(float x, float y) {
    if (x >= 0 and y >= 0 and x < max_dim and y < max_dim and data->at(gridToIndex(x, y)) < 0.5)
        return false;
    return true;
}
int Map::gridToIndex(float x, float y) {
    return (uint)y * max_dim + (uint)x;
}