// pathfinding.cpp : Defines the entry point for the application.
//

#include "pathfinding.h"
#include <chrono>
#include <random>
#define _USE_MATH_DEFINES
#include <math.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/utils/logger.hpp>
#include "map.h"
#include "linear_raycast_pathfinding_strategy.h"
#include "one_tack_pathfinding_strategy.h"
#include "astar_pathfinding_strategy.h"
#include "jump_point_search_pathfinding_strategy.h"
#include "prm_pathfinding_strategy.h"
#include "utilities.h"

#define NOGO_ANGLE_DEGREES 30
#define M_TAU 2*M_PI

using namespace std;

int randomAngleDeg() {
    std::random_device rd;
    std::mt19937 eng(rd());
    std::uniform_int_distribution<> distr(0, 359);
    return distr(eng);
}

bool is_in_nogo(Node* start, Node* goal, float wind_angle_rad, float nogo_angle_rad) {
    auto a = goal->x - start->x;
    auto b = goal->y - start->y;
    double angle_a_b = fmod(atan2(b, a) + M_TAU, M_TAU);

    double opposite_angle = fmod(angle_a_b + M_PI, M_TAU);

    double difference = abs(wind_angle_rad - opposite_angle);
    if (difference > M_PI) {
        difference = M_TAU - difference;
    }
    return(difference < nogo_angle_rad);
}

std::vector<std::pair<double, double>> find_solution(Map map, double wind_angle_deg, Node* start_node, Node* goal_node) {
    double wind_angle_rad = wind_angle_deg * (M_PI / 180);
    double nogo_angle_rad = NOGO_ANGLE_DEGREES * (M_PI / 180);
    bool wind_blocked = false;
    //start with linear solver
    if (!is_in_nogo(start_node, goal_node, wind_angle_rad, nogo_angle_rad)) {
        LinearRaycastPathfindingStrategy linearSolver;
            auto path = linearSolver.solve(map, start_node, goal_node, wind_angle_rad, nogo_angle_rad);
            //return path;
            if (path.size() > 0) {
                displayGrid(map.data, map.max_dim, map.max_dim, path, wind_angle_deg, "grid");
                    return path;
            }
    }
    else {
        wind_blocked = true;
    }
    //if that fails, try one tack
    if (wind_blocked) {
        OneTackPathfindingStrategy oneTackSolver;
        auto path = oneTackSolver.solve(map, start_node, goal_node, wind_angle_rad, nogo_angle_rad);
        if (path.size() > 0) {
            displayGrid(map.data, map.max_dim, map.max_dim, path, wind_angle_deg, "grid");
            return path;
        }
    }
    //if both fail, fall back to pathfinding

    //create solver and solve
    AStarPathfindingStrategy solver;
    /*cout << "Running Astar:" << endl;
    cout << "start: " + to_string(transformed_start_doubles.first) + ", " + to_string(transformed_start_doubles.second) << endl;
    cout << "goal: " + to_string(transformed_goal_doubles.first) + ", " + to_string(transformed_goal_doubles.second) << endl;*/

    auto time_start = std::chrono::high_resolution_clock::now();
    auto path = solver.solve(map, start_node, goal_node);
    auto time_stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(time_stop - time_start);
    cout << "Search time: " + std::to_string(duration.count()) << endl;

    if (path.size() == 0) {
        cout << "No path found" << endl;
        cout << "start: " + std::to_string(start_node->x) + ", " + std::to_string(start_node->y) << std::endl;
        cout << "goal: " + std::to_string(goal_node->x) + ", " + std::to_string(goal_node->y) << std::endl;
        //cout << "transformed start: " + std::to_string(transformed_start_doubles.first) + ", " + std::to_string(transformed_start_doubles.second) << std::endl;
        //cout << "transformed goal: " + std::to_string(transformed_goal_doubles.first) + ", " + std::to_string(transformed_goal_doubles.second) << std::endl;
        return path;
    }

    displayGrid(map.data, map.max_dim, map.max_dim, path, wind_angle_deg, "grid");
    //displayGrid(rotated_map.data, rotated_map.max_dim, rotated_map.max_dim, path_to_doubles(path), 90, "rotated grid");
    return {};
}

void do_maps() {
    Map map = Map(100, 100);
    map.initPRM(100);
    map.generate_obstacles(30, 100);

    cv::Mat grid = cv::Mat(map.max_dim, map.max_dim, CV_32FC1, map.data->data());


    //y direction in openCV is flipped
    double wind_angle_deg = randomAngleDeg();

    auto start = map.randomNode();
    auto goal = map.randomNode();

    find_solution(map, wind_angle_deg, start, goal);

    cv::waitKey(0); // Wait for a key press to close the window
}

int main()
{
    cv::utils::logging::setLogLevel(cv::utils::logging::LogLevel::LOG_LEVEL_SILENT);
    while (true) {
        do_maps();
    }
}

void displayGrid(std::shared_ptr<std::vector<float>> grid, int width, int height, const std::vector<std::pair<double, double>>& path, float windAngleDeg, const char* name) {
    int cellSize = 5; // Size of each cell in the displayed image
    cv::Mat image(height * cellSize, width * cellSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int index = y * width + x;
            if (grid->at(index) > 0.5f) {
                cv::rectangle(image,
                    cv::Point(x * cellSize, y * cellSize),
                    cv::Point((x + 1) * cellSize, (y + 1) * cellSize),
                    cv::Scalar(0, 0, 0),
                    cv::FILLED);
            }
        }
    }

    // Draw the path as a line
    if (path.size() > 0) {
        for (size_t i = 0; i < path.size() - 1; ++i) {
            cv::Point pt1(path[i].first * cellSize + cellSize / 2, path[i].second * cellSize + cellSize / 2);
            cv::Point pt2(path[i + 1].first * cellSize + cellSize / 2, path[i + 1].second * cellSize + cellSize / 2);
            cv::line(image, pt1, pt2, cv::Scalar(0, 0, 255), 2); // Red color for path
        }
    }

    // Draw start position as a green circle
    if (path.size() >= 2) {
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
    }

    cv::Point gridCenter(width * cellSize / 2, height * cellSize / 2);
    // Arrow parameters
    auto windAngle = -windAngleDeg * M_PI / 180;
    int arrowLength = std::min(width, height) * cellSize / 4; // Adjust the length as needed
    cv::Point arrowEnd(
        gridCenter.x + arrowLength * cos(windAngle),
        gridCenter.y - arrowLength * sin(windAngle) // Negative because y-coordinates increase downwards
    );
    cv::arrowedLine(image, gridCenter, arrowEnd, cv::Scalar(0, 255, 0), 2, 8, 0, 0.2);
    cv::Mat flipped;
    cv::flip(image, flipped, 0);
    cv::imshow(name, flipped);
}