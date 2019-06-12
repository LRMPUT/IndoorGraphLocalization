//
// Created by mnowicki on 26.05.19.
//

#include <GraphRoutes.h>

#include "GraphRoutes.h"

double GraphRoutes::euclideanDistance(const std::pair<double, double> &a, const std::pair<double, double> &b) const {
    return sqrt(pow(a.first - b.first, 2) + pow(a.second - b.second, 2));
}

double GraphRoutes::computeHeuristic(const std::pair<double, double> &a, const std::pair<double, double> &b) const {
    return euclideanDistance(a, b);
}


uint64_t GraphRoutes::findClosestNode(const std::pair<double, double> &someNode) const {
    unsigned closestId = 0;
    double closestDist = euclideanDistance(someNode, nodes[closestId]);

    for (int j = 1; j < nodes.size(); j++) {
        double dist = euclideanDistance(someNode, nodes[j]);
        if (dist < closestDist) {
            closestId = j;
            closestDist = dist;
        }
    }
    return closestId;
}

pointProjectionResult GraphRoutes::findClosestProjection(const std::pair<double, double> &someNode) const {
    pointProjectionResult pPR;

    // Best node
    int nodeIdx = findClosestNode(someNode);
    pPR.projectionPoint = nodes[nodeIdx];
    pPR.idx1 = pPR.idx2 = nodeIdx;
    pPR.dist1 = pPR.dist2 = 0.0;
    double bestDist = euclideanDistance(someNode, pPR.projectionPoint);

//    std::cout << "A: " << pPR.projectionPoint.first << " " << pPR.projectionPoint.second << " " << bestDist << std::endl;

    // Finding best projection
    for (uint64_t idx = 0; idx < nodes.size(); idx++) {
        for (uint64_t j = 0; j < neighbours[idx].size(); j++) {
            uint64_t secondIdx = neighbours[idx][j];

            // We have and edge (idx, secondIdx)
            if (secondIdx > idx) {

                auto &a = nodes[idx];
                auto &b = nodes[secondIdx];

                // Line vector
                double px = b.first - a.first;
                double py = b.second - a.second;

                double norm = px * px + py * py;

                // Length of the projection
                double u = ((someNode.first - a.first) * px + (someNode.second - a.second) * py) / norm;

                // If it projects onto the edge
                if (u >= 0 && u <= 1) {
                    // Crossing point
                    double x = a.first + u * px;
                    double y = a.second + u * py;

                    // Distance to crossing point
                    double dx = x - someNode.first;
                    double dy = y - someNode.second;
                    double dist = sqrt(dx*dx+dy*dy);

                    if (dist < bestDist) {
                        pPR.projectionPoint.first = x;
                        pPR.projectionPoint.second = y;
                        pPR.idx1 = idx;
                        pPR.dist1 = euclideanDistance(a, someNode);
                        pPR.idx2 = secondIdx;
                        pPR.dist2 = euclideanDistance(b, someNode);

                        bestDist = dist;
                    }
                }
            }
        }
    }

//    std::cout << "B: " << pPR.projectionPoint.first << " " << pPR.projectionPoint.second << " " << bestDist << std::endl;

    return pPR;
}

uint64_t
GraphRoutes::findNodeWithLowestValue(const std::vector<double> &fValue, const std::vector<uint64_t> &stillOpen) const {

    uint64_t minIdx = stillOpen[0];
    double minVal = fValue[minIdx];


    for (int i = 1; i < stillOpen.size(); i++) {
        uint64_t testIdx = stillOpen[i];
        double testVal = fValue[testIdx];
        if (minVal > testVal) {
            minVal = testVal;
            minIdx = testIdx;
        }
    }

    return minIdx;
}

std::vector<std::pair<double, double>> GraphRoutes::computePath(const std::pair<double, double> &start,
                                                                const std::pair<double, double> &end) const {

    // Closest point on any edge for start
    pointProjectionResult pPR_s = findClosestProjection(start);

    // Closest point on any edge for end
    pointProjectionResult pPR_e = findClosestProjection(end);

    // Points lie on the same edge
    if (pPR_s.idx1 == pPR_e.idx1 && pPR_s.idx2 == pPR_e.idx2) {
//        std::cout << "TUTAJ" << std::endl;
        std::vector<std::pair<double, double> > pathToGoal;
        pathToGoal.push_back(pPR_s.projectionPoint);
        pathToGoal.push_back(pPR_e.projectionPoint);
        return pathToGoal;
    }

    // Points lie on edges that share a vertex
    if (pPR_s.idx1 == pPR_e.idx1 || pPR_s.idx1 == pPR_e.idx2 || pPR_s.idx2 == pPR_e.idx1 || pPR_s.idx2 == pPR_e.idx2)
    {
        int idx = 0;
        if (pPR_s.idx1 == pPR_e.idx1 || pPR_s.idx1 == pPR_e.idx2)
            idx = pPR_s.idx1;
        else
            idx = pPR_s.idx2;

        std::vector<std::pair<double, double> > pathToGoal;
        pathToGoal.push_back(pPR_s.projectionPoint);
        pathToGoal.push_back(nodes[idx]);
        pathToGoal.push_back(pPR_e.projectionPoint);
        return pathToGoal;
    }
    if (pPR_s.idx1 == pPR_e.idx2)
    {
        std::vector<std::pair<double, double> > pathToGoal;
        pathToGoal.push_back(pPR_s.projectionPoint);
        pathToGoal.push_back(nodes[pPR_s.idx1]);
        pathToGoal.push_back(pPR_e.projectionPoint);
        return pathToGoal;
    }
    // find closest node to start
//    uint64_t startIdx = findClosestNode(start);

    // find closest node to end
//    uint64_t endIdx = findClosestNode(end);

//        std::cout << "Idices: " << startIdx << " " << endIdx << std::endl;

    // Initial cost
    std::vector<double> fValue(nodes.size(), 9999999.9), costFromStart(nodes.size(), 9999999.9);
    std::vector<uint64_t> stillOpen, closed;
    std::vector<uint64_t> previousNode(nodes.size(), 0);

    // First node
    fValue[pPR_s.idx1] = pPR_s.dist1 + computeHeuristic(nodes[pPR_s.idx1], pPR_e.projectionPoint);
    fValue[pPR_s.idx2] = pPR_s.dist2 + computeHeuristic(nodes[pPR_s.idx2], pPR_e.projectionPoint);
    costFromStart[pPR_s.idx1] = pPR_s.dist1;
    costFromStart[pPR_s.idx2] = pPR_s.dist2;
    stillOpen.push_back(pPR_s.idx1);
    stillOpen.push_back(pPR_s.idx2);


    // Search for path to goal
    bool goalReached = false;
    while (!goalReached) {

        // Find next node
        uint64_t idx = findNodeWithLowestValue(fValue, stillOpen);

//            std::cout << "Now node : " << idx << std::endl;

        // Updating lists
//        std::cout << stillOpen.size() << std::endl;
        stillOpen.erase(std::remove(stillOpen.begin(), stillOpen.end(), idx), stillOpen.end());
//        std::cout << stillOpen.size() << std::endl;
        closed.push_back(idx);

        // for all edges from this node
        for (int j = 0; j < neighbours[idx].size(); j++) {
            uint64_t secondIdx = neighbours[idx][j];
//                std::cout <<"Neighbour: " << secondIdx << std::endl;

            // Cost from start + edge
            double value = costFromStart[idx] + euclideanDistance(nodes[idx], nodes[secondIdx]);

//                std::cout <<"value: " << value << std::endl;

            // If it is better than previous
            if (value < costFromStart[secondIdx]) {
//                    std::cout <<"better!"<< std::endl;

                costFromStart[secondIdx] = value;

                // Correct prediction to end
                fValue[secondIdx] = value + computeHeuristic(nodes[secondIdx], pPR_e.projectionPoint);

                // Save for path reconstruction
                previousNode[secondIdx] = idx;
            }


            // if it is a goal
            if (secondIdx == pPR_e.idx1 || secondIdx == pPR_e.idx2) {

                // Mark where we came from
                previousNode[secondIdx] = idx;

                goalReached = true;
                break;
            }


            // if it is a new node - it is not already in open and not in closed
            if (std::find(stillOpen.begin(), stillOpen.end(), secondIdx) == stillOpen.end() &&
                std::find(closed.begin(), closed.end(), secondIdx) == closed.end()) {

//                    std::cout <<"Add new node to explore!"<< std::endl;

                // Add for further processing
                stillOpen.push_back(secondIdx);
            }
        }

    }

    // Reconstruct path
    std::vector<std::pair<double, double> > pathToGoal;

    uint64_t nodeIdx = 0;
    if (previousNode[pPR_e.idx1] != 0)
        nodeIdx = pPR_e.idx1;
    else if (previousNode[pPR_e.idx2] != 0)
        nodeIdx = pPR_e.idx2;
    else
        std::cout << "SOMETHING WRONG!" << std::endl;

    pathToGoal.push_back(pPR_e.projectionPoint);
    pathToGoal.push_back(nodes[nodeIdx]);

    while (nodeIdx != pPR_s.idx1 && nodeIdx != pPR_s.idx2) {
        nodeIdx = previousNode[nodeIdx];
        pathToGoal.push_back(nodes[nodeIdx]);
    }
    pathToGoal.push_back(pPR_s.projectionPoint);
    std::reverse(pathToGoal.begin(), pathToGoal.end());

    // TODO: Add additional code if it is not a node
    //

    return pathToGoal;
}
