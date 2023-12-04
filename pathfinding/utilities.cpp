#include "utilities.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <opencv2/opencv.hpp>
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

void drawPRM(std::shared_ptr<std::vector<std::shared_ptr<Node>>> PRMNodes, int maxX, int maxY) {
    // Define image size and padding
    int imageWidth = 800;
    int imageHeight = 800;
    int padding = 0;

    // Scale factors
    float scaleX = float(imageWidth) / float(maxX);
    float scaleY = float(imageHeight) / float(maxY);
    std::cout << "X: " + std::to_string(maxX) << std::endl;
    std::cout << "scale: " + std::to_string(scaleX) << std::endl;

    // Create a white image
    cv::Mat image = cv::Mat::zeros(imageHeight, imageWidth, CV_8UC3);
    image.setTo(cv::Scalar(255, 255, 255));

    // Draw nodes and connections
    for (const auto& node : *PRMNodes) {
        // Scale and translate the node position
        int scaledX = static_cast<int>((node->x) * scaleX) + padding;
        int scaledY = static_cast<int>((node->y) * scaleY) + padding;

        // Draw the node
        cv::circle(image, cv::Point(scaledX, scaledY), 5, cv::Scalar(0, 0, 255), -1);

        // Draw lines to the neighbors
        for (const auto& neighbor : node->neighbors) {
            int neighborX = static_cast<int>((neighbor->x) * scaleX) + padding;
            int neighborY = static_cast<int>((neighbor->y) * scaleY) + padding;
            cv::line(image, cv::Point(scaledX, scaledY), cv::Point(neighborX, neighborY), cv::Scalar(0, 255, 0));
        }
    }

    // Display the image
    cv::imshow("Nodes and Connections", image);
    // Optionally, save the image
    // cv::imwrite("nodes_connections.png", image);
}