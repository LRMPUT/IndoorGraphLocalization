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
#include "graphManager.h"
#include "g2o/core/sparse_optimizer_terminate_action.h"

GraphManager::GraphManager(bool verbose) : verbose(verbose){
    optimizer.setVerbose(verbose);

    // Creating linear solver
    std::unique_ptr<BlockSolverX::LinearSolverType> linearSolver;

    // Cholmod or PCG
    if (cholmod_true_pcg_false)
        linearSolver = g2o::make_unique<LinearSolverCholmod<BlockSolverX::PoseMatrixType>>();
    else
        linearSolver = g2o::make_unique<LinearSolverPCG<BlockSolverX::PoseMatrixType>>();

    // Stop conditions
    g2o::SparseOptimizerTerminateAction* terminateAction = new g2o::SparseOptimizerTerminateAction;
    terminateAction->setGainThreshold(0.0); // 0.0000001
    terminateAction->setMaxIterations(250);
    optimizer.addPostIterationAction(terminateAction);

    // Creating Levenberg algorithm
    OptimizationAlgorithmLevenberg* optimizationAlgorithm = new g2o::OptimizationAlgorithmLevenberg(
            g2o::make_unique<g2o::BlockSolverX>(std::move(linearSolver))
    );

    // Setting Levenberg as optimizaiton algorithm
    optimizer.setAlgorithm(optimizationAlgorithm);
}


double GraphManager::optimize(int iterationCount) {
    if (verbose)
        std::cout << "[GraphManager::optimize] Vertices: " << optimizer.vertices().size() << " Edges: " << optimizer.edges().size() << std::endl;

    // No edges -> return
    if ( optimizer.edges().size() == 0 )
    {
        std::cout << "[GraphManager::optimize] Graph is empty!" << std::endl;
        return 0;
    }

//    // TODO: Test turning off walls
//    for (g2o::HyperGraph::EdgeSet::const_iterator it = optimizer.edges().begin(); it != optimizer.edges().end(); ++it) {
//        EdgeWall *e =dynamic_cast<EdgeWall *>(*it);
//        if (e)
//            e->setLevel(2);
//    }

    // We need to initialize optimization (creating jacobians etc.)
    optimizer.initializeOptimization();

    // Performing optimization
    optimizer.optimize(iterationCount);

//    // TODO: Test turning off walls
//    for (g2o::HyperGraph::EdgeSet::const_iterator it = optimizer.edges().begin(); it != optimizer.edges().end(); ++it) {
//        EdgeWall *e =dynamic_cast<EdgeWall *>(*it);
//        if (e)
//            e->setLevel(0);
//    }
//
//    // We need to initialize optimization (creating jacobians etc.)
//    optimizer.initializeOptimization();
//
//    // Performing optimization
//    optimizer.optimize(iterationCount);





//    for (g2o::HyperGraph::EdgeSet::const_iterator it = optimizer.edges().begin(); it != optimizer.edges().end(); ++it) {
//        EdgeWall *e =dynamic_cast<EdgeWall *>(*it);
//        if (e) {
//
//            double err = e->error()[0];
//
//            if ( err > 0 )
//                std::cout << "Activated: " << e->vertex(0)->id() << " " << e->vertex(1)->id() << " err = " << err << std::endl;
//        }
//    }

    return optimizer.chi2();
}


int GraphManager::saveOptimizationResult(std::string fileName) {
    std::ofstream ofs(fileName);
    int res = optimizer.save(ofs);
    return res;
}


int GraphManager::addVertexPose(const double &x, const double &y, const double &theta, bool fixed) {
    int currentPoseId = nextPoseId++;

    g2o::VertexSE2 *v = new g2o::VertexSE2();
    v->setId(currentPoseId);

    v->setFixed(fixed);

    // Initial estimate
    Eigen::Matrix<double, 3, 1> est;
    est << x, y, theta;
    v->setEstimate(est);

    optimizer.addVertex(v);

    return currentPoseId;
}

void GraphManager::addVertexWiFiLocation(const LocationWiFi &locaWiFi, bool fixed) {
    g2o::VertexSE2 *v = new g2o::VertexSE2();
    v->setId(locaWiFi.locationXY.id);
    v->setFixed(fixed);

    // Initial estimate
    Eigen::Matrix<double, 3, 1> est;
    est << locaWiFi.locationXY.x, locaWiFi.locationXY.y, 0;
    v->setEstimate(est);

    optimizer.addVertex(v);
}

int GraphManager::addVertexStepNodeWithPrior(const double stepPrior, bool fixed, double infMatrixWeight) {
    const int currentStepNodeId = nextStepNodeId++;

    g2o::VertexOne *vOne = new g2o::VertexOne();
    vOne->setId(currentStepNodeId);

    vOne->setFixed(fixed);

    Eigen::Matrix<double, 1, 1> estOne;
    estOne << stepPrior;
    vOne->setEstimate(estOne);
    optimizer.addVertex(vOne);


    g2o::EdgeOnePrior *ePrior = new g2o::EdgeOnePrior();
    ePrior->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(currentStepNodeId)));
    Eigen::Matrix<double,1,1> obs;
    obs << stepPrior;
    ePrior->setMeasurement(obs);
    ePrior->setInformation(Vector1::Identity() * infMatrixWeight);
    optimizer.addEdge(ePrior);

    if (verbose)
        std::cout << "[GraphManager::addStepNode] Successfully added new VertexOne and EdgeOnePrior" << std::endl;

    return currentStepNodeId;
}

void GraphManager::addVerticesForInitialWiFiMap(const std::vector<LocationWiFi> &wifiMap) {
    for (const LocationWiFi &locWiFi : wifiMap) {
        addVertexWiFiLocation(locWiFi, true);
    }
}

void GraphManager::addEdgeWKNN(const int &id, const std::vector<std::pair<double, int>> &wknnWeights, double infMatrixWeight) {

    const int k = wknnWeights.size();
    if (k >= 1) {
        g2o::EdgeWKNN *e = new g2o::EdgeWKNN(k + 1);

        // Localized pose
        g2o::VertexSE2 *v = dynamic_cast<g2o::VertexSE2 *>(optimizer.vertex(id));
        e->setVertex(0, v);

        // Pose guess
        double x = 0, y = 0;

        // Weights contributing to the WKNN
        Eigen::Matrix<double, Eigen::Dynamic, 1> measurement(k, 1);
        for (int i = 0; i < k; i++) {

            int idInGraph = wknnWeights[i].second;
            g2o::VertexSE2 *vWiFi = dynamic_cast<g2o::VertexSE2 *>(optimizer.vertex(idInGraph));

            e->setVertex(i + 1, vWiFi);
            measurement[i] = wknnWeights[i].first;

            x += measurement[i] * vWiFi->estimate().translation().x();
            y += measurement[i] * vWiFi->estimate().translation().y();
        }

        // Override current pose with WKNN guess
        if (assume_initial_pose_from_wknn) {
            Eigen::Matrix<double, 3, 1> est;
            est << x, y, v->estimate().rotation().angle();
            v->setEstimate(est);
        }

        e->setMeasurement(measurement);
        e->setInformation(Eigen::Matrix2d::Identity() * infMatrixWeight);
        optimizer.addEdge(e);

        if (verbose)
            std::cout << "[GraphManager::addEdgeWKNN] Successfully added EdgeWKNN" << std::endl;
    }
}

void GraphManager::addEdgePDR(const int &idPre, const int &idStep, double freqTime, double dangle, double weightXY, double weightTheta,
        std::vector<Wall> walls, double wallPenalty) {

    const g2o::VertexSE2 *vPre = dynamic_cast<const VertexSE2 *>(optimizer.vertex(idPre));
    const g2o::VertexOne *vStep = dynamic_cast<const VertexOne *>(optimizer.vertex(idStep));
    if (vPre && vStep) {

        // Current estimate of the step length
        double estimatedStepLength = vStep->estimate()[0];
        double averageAngle = vPre->estimate()[2] + dangle / 2.0;

        if (verbose)
            std::cout << "PDR: " << estimatedStepLength << " " << dangle / 2.0 * 180.0 / M_PI << std::endl;

        // Guess of the next pose
        // TODO: We should make sure that no wall edge is violated due to local minimums of optimization
        double x = vPre->estimate()[0] + estimatedStepLength * freqTime * cos (averageAngle);
        double y = vPre->estimate()[1] + estimatedStepLength * freqTime * sin (averageAngle);
        double theta = normalize_theta(vPre->estimate()[2] + dangle);

        // Adding new pose
        const int idPost = addVertexPose(x,y,theta,false);

        // Creating PDR edge
        g2o::EdgePDR *e = new g2o::EdgePDR();
        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(idPre)));
        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(idPost)));
        e->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(idStep)));

        // Measurement
        Eigen::Matrix<double, 2, 1> obs2;
        obs2 << freqTime, dangle;
        e->setMeasurement(obs2);

        // Information matrix
        Eigen::Matrix3d informationMatrix = Eigen::Matrix3d::Identity();
        informationMatrix(0,0) = weightXY;
        informationMatrix(1,1) = weightXY;
        informationMatrix(2,2) = weightTheta;
        e->setInformation(informationMatrix);

        // Adding the edge
        optimizer.addEdge(e);

        // TODO: EXPERIMENTAL
//        Eigen::Matrix3d covarianceDist;
//        covarianceDist.setZero();
//        covarianceDist(0,0) = weightXY * cos(averageAngle) * cos(averageAngle);
//        covarianceDist(0,1) = weightXY * cos(averageAngle) * sin(averageAngle);
//        covarianceDist(1,0) = weightXY * cos(averageAngle) * sin(averageAngle);
//        covarianceDist(1,1) = weightXY * sin(averageAngle) * sin(averageAngle);
//
//        Eigen::Vector3d jacobianAngle;
//        jacobianAngle[0] = - 0.5 * estimatedStepLength * freqTime * sin (averageAngle);
//        jacobianAngle[1] = 0.5 * estimatedStepLength * freqTime * cos (averageAngle);
//        jacobianAngle[2] = 1.0;
//        Eigen::Matrix3d covarianceAngle = jacobianAngle * weightTheta * jacobianAngle.transpose();
//
//        Eigen::Matrix3d covariance = covarianceDist + covarianceAngle;
//        informationMatrix = covariance.inverse();
//
//        std::cout << "\tcovariance: " << std::endl << covariance << std::endl;
//
//        std::cout << "\tinformationMatrix: " << std::endl << informationMatrix << std::endl;
//        e->setInformation(informationMatrix);

        // Creating Wall edge
        g2o::EdgeWall *eWall = new g2o::EdgeWall();
        eWall->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(idPre)));
        eWall->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(idPost)));

        // Setting the wall information
        eWall->walls = walls;

        // Measurement
        Eigen::Matrix<double, 1, 1> obsWall;
        obsWall << wallPenalty;
        eWall->setMeasurement(obsWall);

        // Information matrix
        Eigen::Matrix<double, 1, 1> infMatrixWall = Eigen::Matrix<double, 1, 1>::Identity();
        eWall->setInformation(infMatrixWall);

        // Check if it is ok
        for (auto &wall : walls) {

            VertexSE2 *vPost = dynamic_cast<VertexSE2 *>(optimizer.vertex(idPost));
            bool intersect = eWall->doIntersect(wall, vPre, vPost);
            if (intersect) {
                std::cout << "PROBLEM!" << std::endl;
//
//                std::cout << "vPre: " << vPre->estimate()[0] << " " << vPre->estimate()[1] << std::endl;
//                std::cout << "vPost: " << vPost->estimate()[0] << " " << vPost->estimate()[1] << std::endl;


                VertexSE2 *v1 = dynamic_cast<VertexSE2 *>(optimizer.vertex(idPre));

                Eigen::Vector2d wallDirection(wall.endX - wall.startX, wall.endY - wall.startY);
                wallDirection.normalize();

                Eigen::Vector2d userDirection(vPost->estimate()[0] - vPre->estimate()[0],
                                              vPost->estimate()[1] - vPre->estimate()[1]);
                userDirection.normalize();

                double t_nom = (vPre->estimate()[0] - wall.startX) * wallDirection[1] -
                               (vPre->estimate()[1] - wall.startY) * wallDirection[0];
                double t_denom = userDirection[1] * wallDirection[0] - userDirection[0] * wallDirection[1];

                double t = 0.4 * t_nom / t_denom;

                Eigen::Vector2d corectedLocation = Eigen::Vector2d(vPre->estimate()[0], vPre->estimate()[1]) + t * userDirection;

                Eigen::Matrix<double, 3, 1> correctedEstimate;
                correctedEstimate << corectedLocation[0], corectedLocation[1], vPost->estimate()[2];
                vPost->setEstimate(correctedEstimate);



                std::cout << "t: " << t << " userDirection: " <<  userDirection[0] << " " << userDirection[1] << std::endl;
                std::cout << "corectedLocation: " << corectedLocation[0] << " " << corectedLocation[1] << std::endl;
                std::cout << "Wall: (" << wall.startX << " " << wall.startY << ") (" << wall.endX << " " << wall.endY << ")" << std::endl;

                // Let's check
                bool intersect = eWall->doIntersect(wall, vPre, vPost);
                if(intersect)
                    std::cout << "PROBLEM remained !" << std::endl;
                else {
                    std::cout << "PROBLEM was solved !" << std::endl;
                }
//                distance = eWall->distanceToLine(wall, v1, vPost);
//                std::cout << "DISTANCE after correction: " << distance << std::endl;
            }
        }
        // Adding the edge
        optimizer.addEdge(eWall);

        if (verbose)
            std::cout << "[GraphManager::addEdgePDR] Successful added EdgePDR with optional walls" << std::endl;
    }
}

void GraphManager::addEdgeVPR(const int &id, LocationXY imageRecognizedLocation, double weightXY) {

    const g2o::VertexSE2 *vertex = dynamic_cast<const VertexSE2 *>(optimizer.vertex(id));
    if (vertex && imageRecognizedLocation.id >= 0) {

        // Creating VPR edge
        g2o::EdgeVPR *e = new g2o::EdgeVPR();
        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));

        // Measurement
        Eigen::Matrix<double, 2, 1> obs2;
        obs2 << imageRecognizedLocation.x, imageRecognizedLocation.y;
        e->setMeasurement(obs2);

        // Information matrix
        Eigen::Matrix2d informationMatrix = Eigen::Matrix2d::Identity();
        informationMatrix(0,0) = weightXY;
        informationMatrix(1,1) = weightXY;
        e->setInformation(informationMatrix);

        // Adding the edge
        optimizer.addEdge(e);

        if (verbose)
            std::cout << "[GraphManager::addEdgePDR] Successful added EdgeVPR" << std::endl;
    }
}

// Methods to manage ids
int GraphManager::getIdOfLastVertexPose() {
    if (nextPoseId == 0)
        return 0;
    return nextPoseId-1;
}

int GraphManager::getIdofLastVertexStepNode() {
    if (nextStepNodeId == 0)
        return 0;
    return nextStepNodeId-1;
}

// Increase vertex ID for next user
void GraphManager::considerNextUser(int id) {
    nextPoseId = id * NEXT_USER_ID_INCREMENT;

    for (g2o::HyperGraph::EdgeSet::const_iterator it = optimizer.edges().begin(); it != optimizer.edges().end(); ++it) {
        g2o::OptimizableGraph::Edge *e =static_cast<g2o::OptimizableGraph::Edge *>(*it);
        e->setLevel(1);
    }

}

// Optimize everything that was added to the graph
void GraphManager::optimizeAll() {
    for (g2o::HyperGraph::EdgeSet::const_iterator it = optimizer.edges().begin(); it != optimizer.edges().end(); ++it) {
        g2o::OptimizableGraph::Edge *e =static_cast<g2o::OptimizableGraph::Edge *>(*it);
        e->setLevel(0);
    }
    optimize(150);
}

LocationXY GraphManager::getLastPoseEstimate() {
    int id = getIdOfLastVertexPose();


    return getPoseEstimate(id);
}

LocationXY GraphManager::getPoseEstimate(int id) {

    const g2o::VertexSE2 *vertex = dynamic_cast<const VertexSE2 *>(optimizer.vertex(id));
    if(vertex) {
        LocationXY result(vertex->estimate()[0], vertex->estimate()[1], id);
        return result;
    }
    return LocationXY();
}

std::vector<LocationXY> GraphManager::getAllPoseEstimates() {

    std::vector<LocationXY> allPoses;
    for (auto it = optimizer.vertices().begin(); it != optimizer.vertices().end(); ++it) {

        const g2o::VertexSE2 *vertex = dynamic_cast<const VertexSE2 *>(it->second);
        if (vertex) {
            LocationXY result(vertex->estimate()[0], vertex->estimate()[1], it->first);
            allPoses.emplace_back(result);
        }
    }

    return allPoses;
}

std::vector<VisEdge> GraphManager::getAllEdges() {

    std::vector<VisEdge> allEdges;


    for (g2o::HyperGraph::EdgeSet::const_iterator it = optimizer.edges().begin(); it != optimizer.edges().end(); ++it) {

        VisEdge edge;

        EdgeWKNN *e =dynamic_cast<EdgeWKNN *>(*it);
        if (e) {



            const g2o::VertexSE2 *vertex = dynamic_cast<const VertexSE2 *>(e->vertices()[0]);
            edge.startX = vertex->estimate()[0];
            edge.startY = vertex->estimate()[1];

            Eigen::Vector2d wknnEst = e->getWiFiEstimate();
            edge.endX = wknnEst[0];
            edge.endY = wknnEst[1];

            edge.r = 255;
            edge.g = 0;
            edge.b = 0;

            if (verbose)
                std::cout << "GraphManager::getAllEdges() : EdgeWKNN " << edge.startX << " " << edge.startY << " " <<
                        edge.endX << " " << edge.endY << std::endl;



            allEdges.push_back(edge);
        }

        EdgeVPR *evpr =dynamic_cast<EdgeVPR *>(*it);
        if (evpr) {

            const g2o::VertexSE2 *vertex = dynamic_cast<const VertexSE2 *>(evpr->vertices()[0]);
            edge.startX = vertex->estimate()[0];
            edge.startY = vertex->estimate()[1];

            double* vprEst = new double [2];
            evpr->getMeasurementData(vprEst);
            edge.endX = vprEst[0];
            edge.endY = vprEst[1];
            delete [] vprEst;

            edge.r = 0;
            edge.g = 0;
            edge.b = 255;

            if (verbose)
                std::cout << "GraphManager::getAllEdgeVPR() : EdgeVPR" << edge.startX << " " << edge.startY << " " <<
                        edge.endX << " " << edge.endY << std::endl;

            allEdges.push_back(edge);
        }

    }

    return allEdges;
}