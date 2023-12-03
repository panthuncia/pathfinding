#include "utilities.h"
#define _USE_MATH_DEFINES
#include <math.h>
double distance(double x1, double y1, double x2, double y2) {
    double x2x1 = x2 - x1;
    double y2y1 = y2 - y1;
    return sqrt(x2x1 * x2x1 + y2y1 * y2y1);
}
double double_equals(double x, double y, double absTol, double relTol) {
    return (abs(x - y) <= std::max(absTol, relTol * std::max(abs(x), abs(y))));
}

std::pair<double, double> rotateAndScale(Node* pt, double radians, uint32_t h, uint32_t w, uint32_t h_new, uint32_t w_new) {
    double x = pt->x;
    double y = pt->y;
    double offset_x = h / 2;
    double offset_y = w / 2;
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