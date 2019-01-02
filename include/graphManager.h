/*
 * This file is part of the IndoorGraphLocalization distribution (https://github.com/LRMPUT/IndoorGraphLocalization).
 * Copyright (c) 2018 Michał Nowicki (michal.nowicki@put.poznan.pl)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef GRAPHLOCALIZATION_GRAPHMANAGER_H
#define GRAPHLOCALIZATION_GRAPHMANAGER_H

// Core
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/optimizable_graph.h"
#include "g2o/core/base_binary_edge.h"

// Types
#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/types/slam2d/g2o_types_slam2d_api.h"
#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/slam2d/vertex_point_xy.h"
#include "g2o/types/slam3d/vertex_pointxyz.h"

// Solver
#include "g2o/solvers/pcg/linear_solver_pcg.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"

// G2o rest
#include "g2o/stuff/command_args.h"
#include "g2o/config.h"

// Custom edges
#include "local_g2o/edge_pdr.h"
#include "local_g2o/edge_stepometer.h"
#include "local_g2o/edge_wknn.h"
#include "local_g2o/vertex_one.h"
#include "local_g2o/edge_one_prior.h"

#include <mutex>
#include "parameters.h"
#include "wifiLocalization.h"

using namespace std;
using namespace g2o;


class GraphManager {
public:
    GraphManager(bool verbose = false);

    // Perform optimization for given number of iterations
    // Returns chi2 and 0 if not ok (e.g. 0 vertices)
    double optimize(int iterationCount);

    // Save optimized graph to file
    int saveOptimizationResult(std::string fileName);

    // Adds a new pose with (x,y,theta) and fixed value. Returns the id
    int addVertexPose(const double &x, const double &y, const double &theta, bool fixed);

    // Adds a single WiFi location to the optimization
    void addVertexWiFiLocation(const LocationWiFi &locationWiFi, bool fixed);

    // Adds a new vertex to estimate step length. It also adds the step length prior
    int addVertexStepNodeWithPrior(const double stepPrior, bool fixed, double infMatrixWeight);

    // Adds initial WiFi map to the optimization
    void addVerticesForInitialWiFiMap(const std::vector <LocationWiFi> &wifiMap);

    // Adds the WiFi edge for pose id w.r.t the poses saved in wknnWeights (weight, id)
    void addEdgeWKNN(const int &id, const std::vector<std::pair<double, int>> &wknnWeights, double infMatrixWeight);

    // Adds the PDR edge from selected pose (idPre) with step length node (idStep) and measurement (freqTime, dangle)
    void addEdgePDR(const int &idPre, const int &idStep, double freqTime, double dangle, double weightXY, double weightTheta);

    // Methods to manage ids
    int getIdOfLastVertexPose();
    int getIdofLastVertexStepNode();

    // Increase vertex ID for next user
    void considerNextUser(int id);

    // Optimize everything that was added to the graph
    void optimizeAll();

    void getLastVertexPose(double &x, double &y, double &theta) {
        int id = getIdOfLastVertexPose();
        g2o::VertexSE2* v = static_cast<g2o::VertexSE2*>(optimizer.vertex(id));
        x = v->estimate()[0];
        y = v->estimate()[1];
        theta = v->estimate()[2];
        std::cout << "[getLastVertexPose] id: " << id << "| x = " << x << ", y = " << y << ", theta = " << theta << std::endl;
    }

    void getChi2OfEdges() {
        for (g2o::HyperGraph::EdgeSet::const_iterator it = optimizer.edges().begin(); it != optimizer.edges().end(); ++it) {

            g2o::EdgeWKNN *e = dynamic_cast<g2o::EdgeWKNN *>(*it);
            if (e)
                std::cout << "EdgeWKNN: " << e->chi2() << std::endl;


            g2o::EdgePDR *ePDR = dynamic_cast<g2o::EdgePDR *>(*it);
            if (ePDR)
                std::cout << "EdgePDR: " << ePDR->chi2() << std::endl;
        }
    }

private:

    // Optimizer
    SparseOptimizer optimizer;

    // Initial id of the pose (incremented by the addEdgePDR)
    int nextPoseId = INITIAL_USER_ID;

    // Initial id of the step length node (incremented by the addStepNode)
    int nextStepNodeId = STEP_NODE_ID_INCREMENT;
};


#endif //GRAPHLOCALIZATION_GRAPHMANAGER_H
