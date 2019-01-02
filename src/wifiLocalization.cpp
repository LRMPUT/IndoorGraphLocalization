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

#include "wifiLocalization.h"
#include <algorithm>
#include <vector>
#include <utility>
#include <cmath>

static constexpr int ssThreshold = -100;
static constexpr double sharedPercentThreshold = 0.5;

std::pair<double, int> errorL2(const LocationWiFi &lhs, const LocationWiFi &rhs){
    int cnt = 0;
    double diff = 0.0;

    for(const ScanResult &a : lhs.wifiScans){
        for(const ScanResult &b : rhs.wifiScans){
            if(a.level > ssThreshold && b.level > ssThreshold){
                if(a.bssid == b.bssid){
                    diff += (a.level - b.level) * (a.level - b.level);
                    ++cnt;
                }
            }
        }
    }

    if(cnt < 1){
        diff = 1e9;
    }
    else{
        diff = std::sqrt(diff/cnt);
    };
    return std::make_pair(diff, cnt);
}


LocationXY wknn(const std::vector<LocationWiFi> &database,
                const LocationWiFi &scan,
                int k,
                double& meanError)
{
    std::vector<std::pair<double, int>> errors;
    for(int d = 0; d < database.size(); ++d){
        std::pair<double, int> curError = errorL2(scan, database[d]);
        double sharedPercentA = (double) curError.second / scan.wifiScans.size();
        double sharedPercentB = (double) curError.second / database[d].wifiScans.size();

        if (sharedPercentA > sharedPercentThreshold &&
            sharedPercentB > sharedPercentThreshold)
        {
            errors.emplace_back(curError.first, d);
        }
    }
    std::sort(errors.begin(), errors.end());
    double wSum = 0.0;
    LocationXY retLoc(0.0, 0.0, -1);
    meanError = 0.0;
    int meanErrorCnt = 0;
    for(int e = 0; e < std::min(k, (int)errors.size()); ++e){
        double w = 1.0 / (errors[e].first + 0.0001);
        int d = errors[e].second;
        retLoc.x += database[d].locationXY.x * w;
        retLoc.y += database[d].locationXY.y * w;
        wSum += w;

        meanError += errors[e].first;
        ++meanErrorCnt;
    }
    if(wSum > 0.0){
        retLoc.x /= wSum;
        retLoc.y /= wSum;
    }
    if(meanErrorCnt > 0){
        meanError /= meanErrorCnt;
    }
    else{
        meanError = 1e9;
    }

    return retLoc;
}

bool sortByWeights(const std::pair<double,int> &a,
               const std::pair<double,int> &b)
{
    return (a.first > b.first);
}

std::vector<std::pair<double, int>> wknnWeights(const std::vector<LocationWiFi> &database,
                const LocationWiFi &scan,
                int k)
{
    // Computing errors if enough shared networks
    std::vector<std::pair<double, int>> weights;

    for(int d = 0; d < database.size(); ++d){

        // L2 error between scans
        std::pair<double, int> curError = errorL2(scan, database[d]);

        // Shared percent check
        double sharedPercentA = (double) curError.second / scan.wifiScans.size();
        double sharedPercentB = (double) curError.second / database[d].wifiScans.size();

        if (sharedPercentA > sharedPercentThreshold &&
            sharedPercentB > sharedPercentThreshold)
        {
            double weight = 1.0 / (curError.first + 0.0001);
            weights.emplace_back(weight, database[d].locationXY.id);

        }
    }

    // No scans or not enough scans were similar at all
    if ( weights.size() < k ) {
        return std::vector<std::pair<double, int>>();
    }

    // Descending by weights
    std::sort(weights.begin(), weights.end(), sortByWeights);

    // We take only k elements
    if (weights.size() > k)
        weights.resize(k);

    // Normalizing by weights (sum = 1)
    double wSum = 0.0;
    for(int e = 0; e < weights.size(); ++e)
        wSum += weights[e].first;
    for(int e = 0; e < weights.size(); ++e)
        weights[e].first /= wSum;


    return weights;
}