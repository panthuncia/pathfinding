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
    ////start with linear solver
    //if (!is_in_nogo(start_node, goal_node, wind_angle_rad, nogo_angle_rad)) {
    //    LinearRaycastPathfindingStrategy linearSolver;
    //        auto path = linearSolver.solve(map, start_node, goal_node, wind_angle_rad, nogo_angle_rad);
    //        //return path;
    //        if (path.size() > 0) {
    //            displayGrid(map.data, map.max_dim, map.max_dim, path, wind_angle_deg, "grid");
    //                return path;
    //        }
    //}
    //else {
    //    wind_blocked = true;
    //}
    ////if that fails, try one tack
    //if (wind_blocked) {
    //    OneTackPathfindingStrategy oneTackSolver;
    //    auto path = oneTackSolver.solve(map, start_node, goal_node, wind_angle_rad, nogo_angle_rad);
    //    if (path.size() > 0) {
    //        displayGrid(map.data, map.max_dim, map.max_dim, path, wind_angle_deg, "grid");
    //        return path;
    //    }
    //}
    //if both fail, fall back to pathfinding

    //create solver and solve
    AStarPathfindingStrategy solver;
    /*cout << "Running Astar:" << endl;
    cout << "start: " + to_string(transformed_start_doubles.first) + ", " + to_string(transformed_start_doubles.second) << endl;
    cout << "goal: " + to_string(transformed_goal_doubles.first) + ", " + to_string(transformed_goal_doubles.second) << endl;*/

    auto time_start = std::chrono::high_resolution_clock::now();
    auto path = solver.solve(map, start_node, goal_node, wind_angle_rad);
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
    //map.initPRM(500);
    //drawPRM(map.PRMNodes, map.max_dim, map.max_dim);
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