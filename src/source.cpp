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

#include <iostream>


#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/solvers/structure_only/structure_only_solver.h"


#include "local_g2o/vertex_one.h"
#include "local_g2o/edge_one_prior.h"
#include "local_g2o/edge_pdr.h"
#include "local_g2o/edge_wknn.h"

#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/slam2d/vertex_point_xy.h"
#include "g2o/types/slam2d/g2o_types_slam2d_api.h"

#include "DataReadWrite/dataReadWrite.h"
#include "graphManager.h"
#include "wifiLocalization.h"
#include "stepometer.h"

#include "fastable/FastABLE.h"

#include <Eigen/Eigen>
#include <algorithm>



void computeAndAddEdgePDR(GraphManager &graphManager, vector<double> &accWindow, std::vector<std::pair<uint64_t, double>> &accData,
       unsigned int &accIndex, unsigned int &lastAccIndex, bool &isInitialized, std::vector<std::pair<uint64_t, double>> &orientData,
       unsigned int &orientIndex, unsigned int &lastOrientIndex, const std::vector<Wall> &wallMap) {

    double freqTime = Stepometer::computeDist(accWindow,
                                              (accData[accIndex].first - accData[lastAccIndex].first) *
                                              1.0e-9,
                                              accSampFreq,
                                              1,
                                              freqMin,
                                              freqMax,
                                              fftMagThresh);
    lastAccIndex = accIndex;
    accWindow.clear();


    if (isInitialized) {

        // Let's add a PDR edge
        int idStep = graphManager.getIdofLastVertexStepNode();
        int lastVertexPoseId = graphManager.getIdOfLastVertexPose();
//        double orientDiff = -(orientData[orientIndex].second - orientData[lastOrientIndex].second);
//
//        // Normalizing theta
//        if (orientDiff < -M_PI)
//            orientDiff += 2*M_PI;
//        else if (orientDiff > M_PI)
//            orientDiff -= 2*M_PI;

        lastOrientIndex = orientIndex;

        // Do we intent on using orientation in the optimization
//        if (pdr_with_orientation_estimation)
//            graphManager.addEdgePDR(lastVertexPoseId, idStep, freqTime, orientDiff, EDGE_PDR_INF_MAT_METRIC_WEIGHT, EDGE_PDR_INF_MAT_ORIENT_WEIGHT, wallMap, EDGE_WALL_PENALTY);
//        else
            graphManager.addEdgePDR(lastVertexPoseId, idStep, freqTime, 0, EDGE_PDR_INF_MAT_METRIC_WEIGHT, EDGE_PDR_INF_MAT_ORIENT_WEIGHT, wallMap, EDGE_WALL_PENALTY);
    }

}

int main() {

    // Reading running settings
    settings set;
    DataReadWrite::readParameters("parameters.txt", set);

    // Creating am empty graph
    GraphManager graphManager(verbose);

    // Reading initial WiFi map
    double mapImageScale;
//    std::vector<LocationWiFi> wifiMap = DataReadWrite::readMap("dataset/PUTMC_Floor3_Xperia_map", mapImageScale);
    std::vector<LocationImage> imageMap;
    std::vector<LocationWiFi> wifiMap = DataReadWrite::readMap("dataset/2019_04_02_PUTMC_Floor3_Experia_map", mapImageScale, imageMap);

    // Reading information about walls
    std::vector<Wall> wallMap = DataReadWrite::readWalls("dataset/2019_04_02_PUTMC_Floor3_Experia_map");

    // Determines what percent of the original WiFi map we plan on keeping
    DataReadWrite::sparsifyMapPercent(wifiMap, set.mapKeepPercent);

    // Adding WiFi map to graph as fixed
    graphManager.addVerticesForInitialWiFiMap(wifiMap);

    // Initialize FastABLE
    FastABLE fastable(patchSize, compareLength);

    // Add map to FastABLE
    fastable.addImageMap(imageMap);

//    int a;
//    std::cin >> a;

    // Test trajectories
//    std::vector<std::string> testTrajs{"dataset/2019_04_02_PUTMC_Floor3_Experia_trajs/kc_1", "dataset/2019_04_02_PUTMC_Floor3_Experia_trajs/kc_2",
//                                       "dataset/2019_04_02_PUTMC_Floor3_Experia_trajs/kc_3", "dataset/2019_04_02_PUTMC_Floor3_Experia_trajs/kc_4",
//                                       "dataset/2019_04_02_PUTMC_Floor3_Experia_trajs/mn_1", "dataset/2019_04_02_PUTMC_Floor3_Experia_trajs/mn_2",
//                                       "dataset/2019_04_02_PUTMC_Floor3_Experia_trajs/mn_3", "dataset/2019_04_02_PUTMC_Floor3_Experia_trajs/mn_4",
//                                       "dataset/2019_04_02_PUTMC_Floor3_Experia_trajs/mn_5", "dataset/2019_04_02_PUTMC_Floor3_Experia_trajs/mn_sick_1",
//                                       "dataset/2019_04_02_PUTMC_Floor3_Experia_trajs/mn_sick_2", "dataset/2019_04_02_PUTMC_Floor3_Experia_trajs/mn_sick_3",
//                                       "dataset/2019_04_02_PUTMC_Floor3_Experia_trajs/ps_1", "dataset/2019_04_02_PUTMC_Floor3_Experia_trajs/ps_2",
//                                       "dataset/2019_04_02_PUTMC_Floor3_Experia_trajs/ps_3"};
    std::vector<std::string> testTrajs{"dataset/2019_04_02_PUTMC_Floor3_Experia_trajs/mn_1", "dataset/2019_04_02_PUTMC_Floor3_Experia_trajs/mn_2",
                                       "dataset/2019_04_02_PUTMC_Floor3_Experia_trajs/mn_3", "dataset/2019_04_02_PUTMC_Floor3_Experia_trajs/mn_4",
                                       "dataset/2019_04_02_PUTMC_Floor3_Experia_trajs/mn_5"};

    // Processing each trajectory
    std::vector<int> poseCounter(testTrajs.size(), 0), wifiCounter(testTrajs.size(), 0);
    for (int trajIndex=0;trajIndex<testTrajs.size();trajIndex++) {
        int poseNum = 1, wifiNum = 0;

        if (verbose)
            std::cout << "Trajectory id: " << trajIndex << std::endl;

        // Variable indicating if have an initial WiFi fix
        bool isInitialized = false;

        // We increase the ids cause we will localize next user
        if (trajIndex > 0) {
            graphManager.considerNextUser(trajIndex);
        }

        // Reading data from accelerometer, orientation estimation and WiFi adapter
        std::vector<std::pair<uint64_t, double>> accData = DataReadWrite::readAcc(testTrajs[trajIndex]);
        std::vector<std::pair<uint64_t, double>> orientData = DataReadWrite::readOrient(testTrajs[trajIndex]);
        std::vector<LocationImage> testImage;
        std::vector<LocationWiFi> testLocation = DataReadWrite::readMap(testTrajs[trajIndex], mapImageScale, testImage);

        // Indices of samples to be read
        unsigned int accIndex = 0, wifiIndex = 0, orientIndex = 0, lastOrientIndex = 0, lastAccIndex = 0, imageIndex = 0;

        // Accumulated measurements for stepometer
        vector<double> accWindow;

        // WiFi scan with users id (to add it for later processing
        vector<unsigned int> indexOfPosesWithWiFi;
        std::vector<LocationWiFi> successfulWiFiLocations;

        // Simulating motion
        while (true) {

            // We only consider trajectories between WiFi measurements so we end if last WiFi scan was read
            uint64_t wifiTimestamp = std::numeric_limits<uint64_t>::max();
            if (wifiIndex < testLocation.size())
                wifiTimestamp = testLocation[wifiIndex].timestamp;
            else
                break;

            uint64_t accTimestamp = std::numeric_limits<uint64_t>::max();
            if (accIndex < accData.size())
                accTimestamp = accData[accIndex].first;

            uint64_t orientTimestamp = std::numeric_limits<uint64_t>::max();
            if (orientIndex < orientData.size())
                orientTimestamp = orientData[orientIndex].first;

            uint64_t imageTimestamp = std::numeric_limits<uint64_t>::max();
            if (imageIndex < testImage.size())
                imageTimestamp = testImage[imageIndex].timestamp;


            // Finding the lowest timestamp to process
            std::vector<uint64_t> timestamps = {wifiTimestamp, accTimestamp, orientTimestamp, imageTimestamp};
            std::sort(timestamps.begin(), timestamps.end());

            // We processed everything (not needed as WiFi should break earlier)
            if (timestamps[0] == std::numeric_limits<uint64_t>::max())
                break;

            // Process accelerometer
            if (timestamps[0] == accTimestamp) {
//                std::cout << "Acc : " << accTimestamp << std::endl;

                // Something went wrong and over 1 second difference in the original data
                if (accIndex > 0 && (accTimestamp - accData[accIndex-1].first) > 1e9) {
                    accWindow.clear();
                    lastAccIndex = accIndex;
                }

                // Adding for stepometer
                accWindow.push_back(accData[accIndex].second);
                accIndex++;

                // Some movement was made compared to previous location and we have enough samples
                if (accWindow.size() >= winLen) {
                    poseNum++;
                    computeAndAddEdgePDR(graphManager, accWindow, accData, accIndex, lastAccIndex, isInitialized,
                                         orientData, orientIndex, lastOrientIndex, wallMap);
                }
            }
            // Process orientation
            else if (timestamps[0] == orientTimestamp) {
//                std::cout << "Ori : " << orientTimestamp << std::endl;

                // Something went wrong and over 1 second difference
                if (orientIndex > 0 && orientTimestamp - orientData[orientIndex-1].first > 1e9)
                    lastOrientIndex = orientIndex;

                //If significant orientation change was detected we could add additional PDR edge
                if (add_new_vertex_if_significant_orientation_change) {
                    double orientDiff = (orientData[orientIndex].second - orientData[lastOrientIndex].second);
                    if (fabs(orientDiff) > significant_orientation_change_threshold) {
                        computeAndAddEdgePDR(graphManager, accWindow, accData, accIndex, lastAccIndex, isInitialized,
                                             orientData, orientIndex, lastOrientIndex, wallMap);
                    }
                }

                orientIndex++;
            }
            // Process wifi
            else if (timestamps[0] == wifiTimestamp){
//                std::cout << "WiFi: " << wifiTimestamp << std::endl;

                // Localizing with WiFi
                std::vector<std::pair<double, int>> edgeWknnWeights = wknnWeights(wifiMap, testLocation[wifiIndex],
                                                                                  WKNN_N);
                // Successful WiFi localization
                if(edgeWknnWeights.size() > 0 ) {

                    // It was an initial fix
                    if (!isInitialized) {
                        // Adding initial pose (not fixed)
                        graphManager.addVertexPose(0, 0, 0, false);

                        // Adding initial step length
                        graphManager.addVertexStepNodeWithPrior(EDGE_PRIOR_VAL, !set.stepLengthEstimation,
                                                                EDGE_PRIOR_INF_MAT_WEIGHT);

                        isInitialized = true;
                    }
                    // Some movement was made compared to previous location so we create PDR link before WiFi link is added
                    else if (accWindow.size() >= 0) {
                        poseNum++;
                        computeAndAddEdgePDR(graphManager, accWindow, accData, accIndex, lastAccIndex, isInitialized,
                                             orientData, orientIndex, lastOrientIndex, wallMap);
                    }

                    // Adding WiFi measurement. WiFi with deadzone is not used
                    wifiNum++;
                    int lastVertexPoseId = graphManager.getIdOfLastVertexPose();
                    graphManager.addEdgeWKNN(lastVertexPoseId, edgeWknnWeights, EDGE_WKNN_INF_MAT_WEIGHT);
                }

                // We are adding all locations with WiFi to map if chosen. Uncomment to add only correctly localized
                if(edgeWknnWeights.size() > 0 || add_not_localized_wifi_to_map ) {
                    successfulWiFiLocations.push_back(testLocation[wifiIndex]);
                    successfulWiFiLocations.back().locationXY.id = graphManager.getIdOfLastVertexPose();
                }

                wifiIndex++;

                // Optimize as new information is available
                if (online_optimization)
                    graphManager.optimize(250);
            }
            else if (timestamps[0] == imageTimestamp) {
//                std::cout << "Img : " << imageTimestamp << std::endl;

                // TODO: Processing image
                int correctCount = fastable.addNewTestingImage(testImage[imageIndex].image);

                if (correctCount > 0) {
                    std::cout << "trajIndex = " << trajIndex << "  vertexId = " << graphManager.getIdOfLastVertexPose()
                              << "  correctCount = " << correctCount << std::endl;
                }

                imageIndex++;
            }
        }

        // If we decided to add measurement from other trajectories to the optimization
        if (set.interUserConnections) {

            // User was optimized -> lets add his WiFi scans to the map
            for (int i = 0; i < successfulWiFiLocations.size(); i++) {

                // Adding the single location to the WiFi map
                wifiMap.push_back(successfulWiFiLocations[i]);
            }
        }

        // Stats
        poseCounter [trajIndex] = poseNum;
        wifiCounter [trajIndex] = wifiNum;
    }

    // Final optimization of everything
    graphManager.optimizeAll();

    // Saving the resulting graph to the output.g2o
    graphManager.saveOptimizationResult("output.g2o");

    // Graph size stats
    if (verbose) {
        for (int trajIndex = 0; trajIndex < testTrajs.size(); trajIndex++) {
            std::cout << "traj: " << trajIndex + 1 << " \tposes: " << poseCounter[trajIndex] << " (with wifi measurements: "
                      << wifiCounter[trajIndex] << ")"<< std::endl;
        }
    }

    return 0;
}
