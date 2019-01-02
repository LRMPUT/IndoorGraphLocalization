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
#include "DataReadWrite/dataReadWrite.h"
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <utility>
#include <cmath>
#include <Eigen/Eigen>
#include <Eigen/src/Geometry/Quaternion.h>

void DataReadWrite::readParameters(std::string filename, settings &set) {
    std::ifstream file(filename);

    std::string line;
    if (file.is_open()) {

        while (!file.eof()) {
            getline(file, line, ' ');

            if (line == "interUserConnections") {
                getline(file, line);
                if (line == "false")
                    set.interUserConnections = false;
                else
                    set.interUserConnections = true;
            }

            if (line == "stepLengthEstimation") {
                getline(file, line);
                if (line == "false")
                    set.stepLengthEstimation = false;
                else
                    set.stepLengthEstimation = true;
            }

            if (line == "mapKeepPercent") {
                getline(file, line);
                set.mapKeepPercent = std::stod(line);
            }

        }
        file.close();
    }

    std::cout << "Settings: " << std::endl;
    std::cout << "\tinterUserConnections = " << set.interUserConnections << std::endl;
    std::cout << "\tstepLengthEstimation = " << set.stepLengthEstimation << std::endl;
    std::cout << "\tmapKeepPercent = " << set.mapKeepPercent << std::endl;
}

std::vector<LocationWiFi> DataReadWrite::readMap(const std::string &dirPath,
                                                 double &mapImageScale) {
    std::cout << "[DataReadWrite::readMap] Reading map" << std::endl;

    std::vector<LocationWiFi> mapLocations;

    std::string filename = dirPath + "/wifi.map";
    std::ifstream wifiFile(filename.c_str());
    if (!wifiFile.is_open()) {
        std::cout << "[DataReadWrite::readMap] Error! Could not open " << (dirPath + "/wifi.map").c_str() << " file"
                  << std::endl;
    }
    while (!wifiFile.eof() && !wifiFile.fail()) {
        LocationWiFi curLoc;

        uint64_t timestamp = 0;
        int locId = 0;
        int nscans = 0;
        wifiFile >> timestamp >> locId >> nscans;

        curLoc.timestamp = timestamp;
        if (!wifiFile.fail()) {
            for (int i = 0; i < nscans; ++i) {
                std::string bssid;
                std::string ssid;
                double level = 0;
                int freq = 0;
                uint64_t localTimestamp = 0;

                {
                    std::string tmp;
                    std::getline(wifiFile, tmp, '\n');
                }
                std::getline(wifiFile, bssid, '\t');
                std::getline(wifiFile, ssid, '\t');
                wifiFile >> level >> freq >> localTimestamp;\
                if (!wifiFile.fail()) {
                    curLoc.wifiScans.emplace_back(bssid, ssid, level, freq, localTimestamp);
                }
            }

            mapLocations.push_back(curLoc);
        }
    }
    std::cout << "[DataReadWrite::readMap] Read " << mapLocations.size() << " locations" << std::endl;

    std::ifstream positionsFile((dirPath + "/positions.map").c_str());
    if (!positionsFile.is_open()) {
        std::cout << "Error! Could not open " << (dirPath + "/positions.map").c_str() << " file" << std::endl;
    }
    while (!positionsFile.eof() && !positionsFile.fail()) {
        int id;
        double x, y;
        positionsFile >> id >> x >> y;
        if (!positionsFile.fail()) {
            if (id >= 0 && id < mapLocations.size()) {
                mapLocations[id].locationXY = LocationXY(x, y, MAP_ID_INCREMENT + id);
            }
        }
    }

    std::ifstream scaleFile((dirPath + "/scale.map").c_str());
    if (!scaleFile.is_open()) {
        std::cout << "[DataReadWrite::readMap] Error! Could not open " << (dirPath + "/scale.map").c_str() << " file"
                  << std::endl;
    }
    scaleFile >> mapImageScale;


    std::cout << "[DataReadWrite::readMap] Map successfully read" << std::endl;

    return mapLocations;
}


std::vector<std::pair<uint64_t, double>> DataReadWrite::readAcc(const std::string &dirPath) {

    std::cout << "[DataReadWrite::readAcc] Reading acc data" << std::endl;

    std::ifstream accFile((dirPath + "/acc.map").c_str());
    if (!accFile.is_open()) {
        std::cout << "Error! Could not open " << dirPath + "/acc.map" <<
                  std::endl;
    }

    std::vector<std::pair<uint64_t, double>> accData;

    while (!accFile.eof() & !accFile.fail()) {

        uint64_t timestamp;
        double accX, accY, accZ;
        accFile >> timestamp >> accX >> accY >> accZ;
        if (!accFile.fail()) {

            double accMag = std::sqrt(accX * accX + accY * accY + accZ * accZ);
            accData.push_back(std::make_pair(timestamp, accMag));
        }
    }

    std::cout << "[DataReadWrite::readAcc] Acc data successfully read" << std::endl;


    return accData;
}


std::vector<std::pair<uint64_t, double>> DataReadWrite::readOrient(const std::string &dirPath) {

    std::cout << "[DataReadWrite::readOrient] Reading orient data" << std::endl;

    std::ifstream accFile((dirPath + "/orient.map").c_str());
    if (!accFile.is_open()) {
        std::cout << "Error! Could not open " << dirPath + "/orient.map" <<
                  std::endl;
    }

    std::vector<std::pair<uint64_t, double>> orientData;

    while (!accFile.eof() & !accFile.fail()) {

        uint64_t timestamp;
        double qx, qy, qz, qw, headingAcc;
        accFile >> timestamp >> qx >> qy >> qz >> qw >> headingAcc;
        if (!accFile.fail()) {

            Eigen::Quaterniond q(qw, qx, qy, qz);
            Eigen::Matrix3d rotationMatrix = q.toRotationMatrix();

            // Getting the yaw angle from full orientation
            double testAngle = 3.1415265/2.0 - atan2(-rotationMatrix(1,2), rotationMatrix(0,2));
            orientData.push_back(std::make_pair(timestamp, testAngle));
        }
    }

    std::cout << "[DataReadWrite::readOrient] Orient data successfully read" << std::endl;


    return orientData;
}


void DataReadWrite::sparsifyMapPercent(std::vector<LocationWiFi> & wifiMap, double keepPercent) {

    // We leave only keepPercent of original measurements
    int wantedSize = keepPercent * wifiMap.size();

    // Shuffle
    std::shuffle(wifiMap.begin(), wifiMap.end(), std::mt19937{std::random_device{}()});

    // Leave only some elements
    wifiMap.resize(wantedSize);

}
