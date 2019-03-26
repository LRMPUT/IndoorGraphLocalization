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
#ifndef GRAPHLOCALIZATION_DATAREADWRITE_H
#define GRAPHLOCALIZATION_DATAREADWRITE_H

#include "wifiLocalization.h"
#include <vector>
#include <string>
#include <iostream>
#include <fstream>

#include <algorithm>
#include <numeric>
#include <random>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>

class DataReadWrite {
public:
    static void readParameters(std::string filename, settings &set);

    static std::vector<LocationWiFi> readMap(const std::string &dirPath,
                                             double &mapImageScale,
                                             std::vector<LocationImage> &images);


    static std::vector<std::pair<uint64_t, double>> readAcc(const std::string &dirPath);

    static std::vector<std::pair<uint64_t, double>> readOrient(const std::string &dirPath);

    static std::vector<Wall> readWalls(const std::string &dirPath);

    static void sparsifyMapPercent(std::vector<LocationWiFi> & wifiMap, double keepPercent);
};


#endif //GRAPHLOCALIZATION_DATAREADWRITE_H
