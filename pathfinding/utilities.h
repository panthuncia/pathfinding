#pragma once
#include <cmath>
#include <memory>
#include "node.h"
#include "map.h"
#define NOGO_ANGLE_DEGREES 45
double double_equals(double x, double y, double absTol = 0.000000001, double relTol = 0.000000001);
double distance(double x1, double y1, double x2, double y2);
std::pair<double, double> rotateAndScale(Node* pt, double radians, uint32_t h, uint32_t w, uint32_t h_new, uint32_t w_new);
std::vector<std::pair<double, double>> rotate_path_doubles(std::vector<Node*> path, uint32_t oldHeight, uint32_t oldWidth, uint32_t newHeight, uint32_t newWidth, double angle_deg);
std::vector<std::pair<double, double>> path_to_doubles(std::vector<Node*> path);
std::vector<std::pair<double, double>> simplify_path(std::vector<std::pair<double, double>> originalPath, double wind_angle_deg, Map& map);
int randomAngleDeg();
bool is_in_nogo(Node* start, Node* goal, float wind_angle_rad, float nogo_angle_rad);
bool is_in_nogo(std::pair<double, double> start, std::pair<double, double> goal, float wind_angle_rad, float nogo_angle_rad);
void displayGrid(std::shared_ptr<std::vector<float>> grid, int width, int height, const std::vector<std::pair<double, double>>& path, float windAngle, const char* name);
void drawPRM(std::shared_ptr<std::vector<Node*>> PRMNodes, int maxX, int maxY);