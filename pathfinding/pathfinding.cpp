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

#define NOGO_ANGLE_DEGREES 30
#define M_TAU 2*M_PI

using namespace std;
Node generateRandomPoint(int width, int height) {
    std::random_device rd;
    std::mt19937 eng(rd());
    std::uniform_int_distribution<> distrX(0, width - 1);
    std::uniform_int_distribution<> distrY(0, height - 1);

    return Node(distrX(eng), distrY(eng));
}

int randomAngleDeg() {
    std::random_device rd;
    std::mt19937 eng(rd());
    std::uniform_int_distribution<> distr(0, 359);
    return distr(eng);
}


std::pair<double, double> rotateAndScale(Node* pt, double radians, uint32_t h, uint32_t w, uint32_t h_new, uint32_t w_new) {
    double x = pt->x;
    double y = pt->y;
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

std::vector<std::pair<double, double>> rotate_path_doubles(std::vector<Node*> path, uint32_t oldHeight, uint32_t oldWidth, uint32_t newHeight, uint32_t newWidth, double angle_deg) {
    std::vector<std::pair<double, double>> transformed_path;
    for (Node* n : path) {
        auto transformed_doubles = rotateAndScale(n, -angle_deg * (M_PI / 180), oldHeight, oldWidth, newHeight, newWidth);
        transformed_path.push_back(std::make_pair(transformed_doubles.first, transformed_doubles.second));
    }
    return transformed_path;
}

std::vector<std::pair<double, double>> path_to_doubles(std::vector<Node*> path) {
    std::vector<std::pair<double, double>> doubles;
    for (Node* n : path) {
        doubles.push_back(std::make_pair(n->x, n->y));
    }
    return doubles;
}


bool is_in_nogo(Node* start, Node* goal, float wind_angle_rad, float nogo_angle_rad) {
    auto a = goal->x - start->x;
    auto b = goal->y - start->y;
    double angle_a_b = fmod(atan2(b, a) + M_TAU, M_TAU);
    std::cout << angle_a_b << std::endl;

    double opposite_angle = fmod(angle_a_b + M_PI, M_TAU);

    double difference = abs(wind_angle_rad - opposite_angle);
    if (difference > M_PI) {
        difference = M_TAU - difference;
    }
    std::cout << difference << std::endl;
    return(difference < nogo_angle_rad);
}

std::vector<Node*> find_solution(Map map, double wind_angle_deg, Node start, Node goal) {
    double wind_angle_rad = wind_angle_deg * (M_PI / 180);
    double nogo_angle_rad = NOGO_ANGLE_DEGREES * (M_PI / 180);
    bool wind_blocked = false;
    //start with linear solver
    auto start_node = map.getNode(start.x, start.y);
    auto goal_node = map.getNode(goal.x, goal.y);
    if (!is_in_nogo(start_node, goal_node, wind_angle_rad, nogo_angle_rad)) {
        LinearRaycastPathfindingStrategy linearSolver;
            auto path = linearSolver.solve(map, map.getNode(start.x, start.y), map.getNode(goal.x, goal.y), wind_angle_rad, nogo_angle_rad);
            //return path;
            if (path.size() > 0) {
                displayGrid(map.data, map.width, map.height, path_to_doubles(path), wind_angle_deg, "grid");
                    return path;
            }
    }
    else {
        wind_blocked = true;
    }
    //if that fails, try one tack
    if (wind_blocked) {
        OneTackPathfindingStrategy oneTackSolver;
        auto path = oneTackSolver.solve(map, map.getNode(start.x, start.y), map.getNode(goal.x, goal.y), wind_angle_rad, nogo_angle_rad);
        if (path.size() > 0) {
            displayGrid(map.data, map.width, map.height, path_to_doubles(path), wind_angle_deg, "grid");
            return path;
        }
    }
    //if both fail, fall back to pathfinding
    //rotate map to enable wind restriction
    double map_angle_deg = -90 + wind_angle_deg;
    auto rotate_time_start = std::chrono::high_resolution_clock::now();
    Map rotated_map = map.rotate(map_angle_deg);
    auto rotate_time_stop = std::chrono::high_resolution_clock::now();
    auto rotate_duration = std::chrono::duration_cast<std::chrono::milliseconds>(rotate_time_stop - rotate_time_start);
    cout << "full rotate time: " + std::to_string(rotate_duration.count()) << endl;


    auto transformed_start_doubles = rotateAndScale(&start, map_angle_deg, map.height, map.width, rotated_map.height, rotated_map.width);
    auto transformed_goal_doubles = rotateAndScale(&goal, map_angle_deg, map.height, map.width, rotated_map.height, rotated_map.width);

    //create solver and solve
    AStarPathfindingStrategy solver;
    cout << "start: " + to_string(transformed_start_doubles.first) + ", " + to_string(transformed_start_doubles.second) << endl;
    cout << "goal: " + to_string(transformed_goal_doubles.first) + ", " + to_string(transformed_goal_doubles.second) << endl;
    auto time_start = std::chrono::high_resolution_clock::now();
    auto path = solver.solve(rotated_map, rotated_map.getNode(transformed_start_doubles.first, transformed_start_doubles.second), rotated_map.getNode(transformed_goal_doubles.first, transformed_goal_doubles.second));
    auto time_stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(time_stop - time_start);

    //diagnostics
    cout << "Search time: " + std::to_string(duration.count()) << endl;
    if (path.size() == 0) {
        cout << "start: " + std::to_string(start.x) + ", " + std::to_string(start.y) << std::endl;
        cout << "goal: " + std::to_string(goal.x) + ", " + std::to_string(goal.y) << std::endl;
        cout << "transformed start: " + std::to_string(transformed_start_doubles.first) + ", " + std::to_string(transformed_start_doubles.second) << std::endl;
        cout << "transformed goal: " + std::to_string(transformed_goal_doubles.first) + ", " + std::to_string(transformed_goal_doubles.second) << std::endl;
        return path;
    }
    displayGrid(map.data, map.width, map.height, rotate_path_doubles(path, rotated_map.height, rotated_map.width, map.height, map.width, map_angle_deg), wind_angle_deg, "grid");
    displayGrid(rotated_map.data, rotated_map.width, rotated_map.height, path_to_doubles(path), 90, "rotated grid");
    return {};
}

void do_maps() {
    Map map = Map(1000, 1000);

    map.generate_obstacles(300, 1000);

    cv::Mat grid = cv::Mat(map.max_dim, map.max_dim, CV_32FC1, map.data->data());
    cv::imshow("image", grid);

    //displayGrid(map.data, map.max_dim, map.max_dim, {}, 0, "grid");



    double wind_angle_deg = 10;// randomAngleDeg();
    double map_angle_deg = -90 + wind_angle_deg;
    auto rotate_time_start = std::chrono::high_resolution_clock::now();
    Map rotated_map = map.rotate(map_angle_deg);
    auto rotate_time_stop = std::chrono::high_resolution_clock::now();
    auto rotate_duration = std::chrono::duration_cast<std::chrono::milliseconds>(rotate_time_stop - rotate_time_start);
    cout << "full rotate time: " + std::to_string(rotate_duration.count()) << endl;
    displayGrid(rotated_map.data, rotated_map.max_dim, rotated_map.max_dim, {}, 0, "rotated grid");
    //auto start = generateRandomPoint(map.width, map.height);
    //auto goal = generateRandomPoint(map.width, map.height);
    //find_solution(map, wind_angle_deg, start, goal);
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
    int cellSize = 1; // Size of each cell in the displayed image
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
    auto windAngle = fmod((-windAngleDeg+180), 360) * M_PI / 180;
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