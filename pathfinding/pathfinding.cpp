﻿// pathfinding.cpp : Defines the entry point for the application.
//

#include "pathfinding.h"
#include <random>
#define _USE_MATH_DEFINES
#include <math.h>
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

std::pair<double, double> rotateAndScale(Node pt, double radians, uint32_t h, uint32_t w, uint32_t h_new, uint32_t w_new) {
    double x = pt.x;
    double y = pt.y;
    double offset_x = h/2;
    double offset_y = w/2;
    double adjusted_x = x - offset_x;
    double adjusted_y = y - offset_y;
    double cos_rad = cos(radians);
    double sin_rad = sin(radians);
    double qx = offset_x + cos_rad * adjusted_x + sin_rad * adjusted_y;
    double qy = offset_y + -sin_rad * adjusted_x + cos_rad * adjusted_y;
    double xoffset = (int(w_new) - int(w)) / 2;
    double yoffset = (int(h_new) - int(h)) / 2;
    double x1_new = qx + xoffset;
    double y1_new = qy + yoffset;
    return std::make_pair(x1_new, y1_new);
}

void do_maps() {
    Map map = Map(100, 100);
    map.generate_obstacles(30, 100);

    //rotate map
    double wind_angle_deg = 30;
    double map_angle_deg = 90 + wind_angle_deg;
    Map rotated_map = map.rotate(map_angle_deg);


    auto start = generateRandomPoint(map.width, map.height);
    auto goal = generateRandomPoint(map.width, map.height);
    auto transformed_start_doubles = rotateAndScale(start, map_angle_deg, map.height, map.width, rotated_map.height, rotated_map.width);
    auto transformed_goal_doubles = rotateAndScale(goal, map_angle_deg, map.height, map.width, rotated_map.height, rotated_map.width);
    Node transformed_start = Node(transformed_start_doubles.first, transformed_start_doubles.second);
    Node transformed_goal = Node(transformed_goal_doubles.first, transformed_goal_doubles.second);
    cout << map.data.size();
    AStarPathfindingStrategy solver;
    auto path = solver.solve(rotated_map, transformed_start, transformed_goal);
    if (path.size() == 0) {
        cout << "start: " + std::to_string(start.x) + ", " + std::to_string(start.y) << std::endl;
        cout << "goal: " + std::to_string(goal.x) + ", " + std::to_string(goal.y) << std::endl;
        cout << "transformed start: " + std::to_string(transformed_start.x) + ", " + std::to_string(transformed_start.y) << std::endl;
        cout << "transformed goal: " + std::to_string(transformed_goal.x) + ", " + std::to_string(transformed_goal.y) << std::endl;
        return;
    }
    std::vector<std::pair<double, double>> transformed_path;
    std::vector<std::pair<double, double>> un_transformed_path;
    for (Node n : path) {
        auto transformed_doubles = rotateAndScale(n, -map_angle_deg * (M_PI / 180), rotated_map.height, rotated_map.width, map.height, map.width);
        un_transformed_path.push_back(std::make_pair(transformed_doubles.first, transformed_doubles.second));
        transformed_path.push_back(std::make_pair(n.x, n.y));
    }
    cout << "length:";
    cout << un_transformed_path.size();
    displayGrid(map.data, map.width, map.height, un_transformed_path, "grid");
    displayGrid(rotated_map.data, rotated_map.width, rotated_map.height, transformed_path, "rotated grid");
    cv::waitKey(0); // Wait for a key press to close the window
}

int main()
{
    while (true) {
        do_maps();
    }
}

void displayGrid(const std::vector<float>& grid, int width, int height, const std::vector<std::pair<double, double>>& path, const char* name) {
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

    // Draw the path as a line
    for (size_t i = 0; i < path.size() - 1; ++i) {
        cv::Point pt1(path[i].first * cellSize + cellSize / 2, path[i].second * cellSize + cellSize / 2);
        cv::Point pt2(path[i + 1].first * cellSize + cellSize / 2, path[i + 1].second * cellSize + cellSize / 2);
        cv::line(image, pt1, pt2, cv::Scalar(0, 0, 255), 2); // Red color for path
    }

    // Draw start position as a green circle
    cv::circle(image,
        cv::Point(path[0].first * cellSize + cellSize / 2, path[0].second * cellSize + cellSize / 2),
        cellSize * 2,
        cv::Scalar(0, 255, 0), // Green color for start
        cv::FILLED);

    // Draw end position as a blue circle
    cv::circle(image,
        cv::Point(path.back().first * cellSize + cellSize / 2, path.back().second * cellSize + cellSize / 2),
        cellSize * 2,
        cv::Scalar(255, 0, 0), // Blue color for end
        cv::FILLED);

    cv::imshow(name, image);
}