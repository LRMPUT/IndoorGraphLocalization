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
#ifndef INDOORGRAPHLOCALIZATION_PARAMETERS_H
#define INDOORGRAPHLOCALIZATION_PARAMETERS_H

// WKNN
#define WKNN_N 4

// The IDs in graph:
// from 0 -> userPoses
// from 10000 -> next UserPose
// from 90 000 -> step nodes
// from 100 000 -> map ids
#define INITIAL_USER_ID 0
#define NEXT_USER_ID_INCREMENT 1000
#define STEP_NODE_ID_INCREMENT 90000
#define MAP_ID_INCREMENT 100000

// STEPOMETER
static constexpr int winLen = 256;
static constexpr double accSampFreq = 200;
static constexpr double freqMin = 1.3;
static constexpr double freqMax = 2.2;
static constexpr double fftMagThresh = 0.2;

// FASTABLE
static constexpr double patchSize = 64;
static constexpr int compareLength = 50;
static constexpr double safetyThresholdRatio = 0.9;

// Graph edges
#define EDGE_PRIOR_VAL 0.65 // Assumed prior step length - 0.65 meters
#define EDGE_PRIOR_INF_MAT_WEIGHT 200 // Weight assigned to the prior step length
#define EDGE_WKNN_INF_MAT_WEIGHT 10 // Weight assigned to the WiFi
#define EDGE_PDR_INF_MAT_METRIC_WEIGHT 1 // Weight assigned to metric part of the PDR
#define EDGE_PDR_INF_MAT_ORIENT_WEIGHT 15 // Weight assigned to orientation part of the PDR
#define EDGE_WALL_PENALTY 0 // Penalty weight of EDGE_WALL

// Running settings
struct settings {
    bool interUserConnections;
    bool stepLengthEstimation;
    double mapKeepPercent;
};

// Additional parameters
static constexpr bool pdr_with_orientation_estimation = true;
static constexpr bool add_not_localized_wifi_to_map = true;
static constexpr bool online_optimization = false;
static constexpr bool add_new_vertex_if_significant_orientation_change = false;
static constexpr double significant_orientation_change_threshold = 45.0 * 3.1415265 / 180.0;
static constexpr bool assume_initial_pose_from_wknn = false;
static constexpr bool cholmod_true_pcg_false = true;

static constexpr bool verbose = false;

#endif //INDOORGRAPHLOCALIZATION_PARAMETERS_H
