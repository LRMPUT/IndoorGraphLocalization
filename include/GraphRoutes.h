//
// Created by mnowicki on 22.05.19.
//

#ifndef GRAPHLOCALIZATION_GRAPHSTRUCTURE_H
#define GRAPHLOCALIZATION_GRAPHSTRUCTURE_H

#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>

struct pointProjectionResult {
    std::pair<double, double> projectionPoint;
    int idx1, idx2;
    double dist1, dist2;
};

class GraphRoutes {
public:

    double euclideanDistance(const std::pair<double, double> &a, const std::pair<double, double> &b) const;

    double computeHeuristic(const std::pair<double, double> &a, const std::pair<double, double> &b) const;


    uint64_t findClosestNode(const std::pair<double, double> &someNode) const;

    pointProjectionResult findClosestProjection(const std::pair<double, double> &someNode) const;

    uint64_t findNodeWithLowestValue(const std::vector<double> &fValue, const std::vector<uint64_t> &stillOpen) const;

    std::vector<std::pair<double, double>> computePath(const std::pair<double, double> &start,
                                                       const std::pair<double, double> &end) const;

    double distanceToLine(std::pair<double, double> &p1, std::pair<double, double> &p2, std::pair<double, double> &q) {


    }

    std::vector<std::pair<double, double>> nodes;
    std::vector<std::vector<uint64_t>> neighbours;

};

#endif //GRAPHLOCALIZATION_GRAPHSTRUCTURE_H
