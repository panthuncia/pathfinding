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
    auto start = generateRandomPoint(map.width, map.height);
    auto goal = generateRandomPoint(map.width, map.height);
    cout << map.data.size();
    AStarPathfindingStrategy solver;
    auto path = solver.solve(map, start, goal);
    cout << "length:";
    cout << path.size();
    displayGrid(map.data, map.width, map.height, path);
	return 0;
}

void displayGrid(const std::vector<float>& grid, int width, int height, const std::vector<Node>& path) {
    int cellSize = 10; // Size of each cell in the displayed image
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

    cv::imshow("Grid", image);
    cv::waitKey(0); // Wait for a key press to close the window
}