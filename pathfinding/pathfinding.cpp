// pathfinding.cpp : Defines the entry point for the application.
//

#include "pathfinding.h"
#include <random>
#include <opencv2/opencv.hpp>
#include "map.h"
#include "astar_pathfinding_strategy.h"
using namespace std;
Node generateRandomPoint(int width, int height) {
    std::random_device rd;
    std::mt19937 eng(rd());
    std::uniform_int_distribution<> distrX(0, width - 1);
    std::uniform_int_distribution<> distrY(0, height - 1);

    return Node(distrX(eng), distrY(eng));
}

int main()
{
    Map map = Map(100, 100, 30, 100);

    //rotate map
    cv::Mat grid = cv::Mat(map.height, map.width, CV_32FC1, map.data.data());
    double wind_angle_deg = 30;
    double map_angle_deg = 90 + wind_angle_deg;
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

    auto start = generateRandomPoint(map.width, map.height);
    auto goal = generateRandomPoint(map.width, map.height);
    cout << map.data.size();
    AStarPathfindingStrategy solver;
    auto path = solver.solve(map, start, goal);
    cout << "length:";
    cout << path.size();
    displayGrid(map.data, map.width, map.height, path, "grid");
    displayGrid(rotated_vector, newWidth, newHeight, path, "rotated grid");

    cv::waitKey(0); // Wait for a key press to close the window
	return 0;
}

void displayGrid(const std::vector<float>& grid, int width, int height, const std::vector<Node>& path, const char* name) {
    int cellSize = 3; // Size of each cell in the displayed image
    cv::Mat image(height * cellSize, width * cellSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int index = y * width + x;
            if (grid[index] == 1.0f) {
                cv::rectangle(image,
                    cv::Point(x * cellSize, y * cellSize),
                    cv::Point((x + 1) * cellSize, (y + 1) * cellSize),
                    cv::Scalar(0, 0, 0),
                    cv::FILLED);
            }
        }
    }

    for (const auto& node : path) {
        int index = node.y * width + node.x;
        cv::rectangle(image,
            cv::Point(node.x * cellSize, node.y * cellSize),
            cv::Point((node.x + 1) * cellSize, (node.y + 1) * cellSize),
            cv::Scalar(0, 0, 255), // Red color for path
            cv::FILLED);
    }

    cv::imshow(name, image);
}