#include <Eigen/Core>
#include <Eigen/Geometry>
#include <obj.h>
#include <iostream>
#include <vector>
#include <random>
#include <algorithm>

namespace tnp {

float pointPlaneDistance(const Eigen::Vector3f& point, const Eigen::Vector3f& planePoint, const Eigen::Vector3f& planeNormal) {
    return fabs((point - planePoint).dot(planeNormal.normalized()));
}

void selectRandomPoints(const std::vector<Eigen::Vector3f>& points, std::vector<Eigen::Vector3f>& selectedPoints) {
    unsigned long n = 3;
    if (points.size() <= n) {
        selectedPoints = points;
        return;
    }

    std::random_device rd;
    std::mt19937 gen(rd());

    selectedPoints.insert(selectedPoints.end(), points.begin(), points.begin() + n);

    for (size_t i = n; i < points.size(); ++i) {
        std::uniform_int_distribution<> dis(0, i);
        unsigned long j = dis(gen);

        if (j < n) {
            selectedPoints[j] = points[i];        }
    }
}

float angleBetweenNormals(const Eigen::Vector3f& normal1, const Eigen::Vector3f& normal2) {
    float dotProduct = normal1.normalized().dot(normal2.normalized());
    float angle = acos(std::max(-1.0f, std::min(1.0f, dotProduct)));
    return angle;
}

// compute a plane from 3 points
void computePlane(const std::vector<Eigen::Vector3f>& selectedPoints, Eigen::Vector3f& planePoint, Eigen::Vector3f& planeNormal) {
    Eigen::Vector3f v1 = selectedPoints[1] - selectedPoints[0];
    Eigen::Vector3f v2 = selectedPoints[2] - selectedPoints[0];
    planeNormal = v1.cross(v2);
    planePoint = selectedPoints[0];
}

void RANSAC( const std::vector<Eigen::Vector3f>& points, const std::vector<Eigen::Vector3f>& normals, Eigen::Vector3f& bestPlanePoint, 
    Eigen::Vector3f& bestPlaneNormal, int iterations, float distanceThreshold, float angleThreshold, bool isnormals) {
    int bestCount = 0;

    for (int i = 0; i < iterations; ++i) {
        std::vector<Eigen::Vector3f> selectedPoints;
        selectRandomPoints(points, selectedPoints);

        Eigen::Vector3f planePoint, planeNormal;
        computePlane(selectedPoints, planePoint, planeNormal);

        int count = 0;
        for (size_t j = 0; j < points.size(); ++j) {
            if (pointPlaneDistance(points[j], planePoint, planeNormal) < distanceThreshold) {
                if (isnormals) { 
                    if (angleBetweenNormals(normals[j], planeNormal) < angleThreshold) {
                        ++count;
                    }
                }
                else {
                    ++count;
                }
            }
        }

        if (count > bestCount) {
            bestCount = count;
            bestPlanePoint = planePoint;
            bestPlaneNormal = planeNormal;
        }
    }
}


void removeClosePoints(std::vector<Eigen::Vector3f>& points, const Eigen::Vector3f& planePoint, const Eigen::Vector3f& normalizedPlaneNormal, float threshold) {
    auto newEnd = std::remove_if(points.begin(), points.end(),
                                 [&planePoint, &normalizedPlaneNormal, threshold](const Eigen::Vector3f& point) {
                                     return pointPlaneDistance(point, planePoint, normalizedPlaneNormal) < threshold;
                                 });
    points.erase(newEnd, points.end());
}


void removeClosePoints(std::vector<Eigen::Vector3f>& points, std::vector<Eigen::Vector3f>& normals, const Eigen::Vector3f& planePoint, const Eigen::Vector3f& normalizedPlaneNormal, float threshold) {
    std::vector<size_t> indicesToRemove;

    for (size_t i = 0; i < points.size(); ++i) {
        if (pointPlaneDistance(points[i], planePoint, normalizedPlaneNormal) < threshold) {
            indicesToRemove.push_back(i);
        }
    }

    for (auto it = indicesToRemove.rbegin(); it != indicesToRemove.rend(); ++it) {
        points.erase(points.begin() + *it);
        normals.erase(normals.begin() + *it);
    }
}



}


int main(int argc, char const *argv[])
{
    if (argc <= 2) {
        std::cout << "Error: missing argument for number of planes" << std::endl;
        std::cout << "Usage: ransac <filename>.obj <number_of_planes>" << std::endl;
        return 0;
    }
    unsigned long numPlanes = std::stoi(argv[2]);

    std::vector<Eigen::Vector3f> points, normals;
    const std::string filename = argv[1];

    if (!tnp::load_obj(filename, points, normals)) {
        std::cout << "Failed to open input file '" << filename << "'" << std::endl;
        return 1;
    }

    bool useNormals = false;
    if (argc > 3 && std::string(argv[3]) == "normals") {
        useNormals = true;
    }

    if ((useNormals) && (points.size() != normals.size())) {
        std::cout << "Vector points of size and normals not the same size" << std::endl;
        return 1;
    }

    std::vector<Eigen::Vector3f> planeColors = {
        Eigen::Vector3f(1.0f, 0.0f, 0.0f), // Red
        Eigen::Vector3f(0.0f, 1.0f, 0.0f), // Green
        Eigen::Vector3f(0.0f, 0.0f, 1.0f), // Blue
        Eigen::Vector3f(1.0f, 1.0f, 0.0f), // Yellow
        Eigen::Vector3f(1.0f, 0.0f, 1.0f), // Magenta
        Eigen::Vector3f(0.0f, 1.0f, 1.0f), // Cyan
        Eigen::Vector3f(0.5f, 0.0f, 0.0f), // Dark Red
        Eigen::Vector3f(0.5f, 0.5f, 0.5f), // Gray
        Eigen::Vector3f(1.0f, 0.5f, 0.0f), // Orange
        Eigen::Vector3f(0.0f, 0.5f, 0.5f)  // Dark Cyan
    };


    int iterations = 100;
    float threshold = 0.1f;
    float angleThreshold = 10;

    std::vector<Eigen::Vector3f> allPoints, allColors;

    std::vector<Eigen::Vector3f> remainingPoints = points;
    std::vector<Eigen::Vector3f> remainingNormals = normals;

    for (unsigned long plane = 0; plane < numPlanes; ++plane) {
        std::cout << "Currently on plane" << plane + 1 << std::endl;
        Eigen::Vector3f bestPlanePoint, bestPlaneNormal;
        tnp::RANSAC(remainingPoints, remainingNormals, bestPlanePoint, bestPlaneNormal, iterations, threshold, angleThreshold, useNormals);

        for (const auto& point : points) {
            if (tnp::pointPlaneDistance(point, bestPlanePoint, bestPlaneNormal) < threshold) {
                allPoints.push_back(point);
                allColors.push_back(planeColors[plane % 10]);
            }
        }
        if (plane + 1 >= numPlanes)
        {
            break;
        }
        if (useNormals) {
            tnp::removeClosePoints(remainingPoints, remainingNormals, bestPlanePoint, bestPlaneNormal, threshold);
        }
        else {
            tnp::removeClosePoints(remainingPoints, bestPlanePoint, bestPlaneNormal, threshold);
        }
    }

    tnp::save_obj("colored_planes.obj", allPoints, {}, allColors);

    return 0;
}
