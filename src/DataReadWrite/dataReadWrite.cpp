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
                                                 double &mapImageScale,
                                                 std::vector<LocationImage> &imageLocations) {
    std::cout << "[DataReadWrite::readMap] Reading map" << std::endl;

    // Reading WiFi
    std::vector<LocationWiFi> wifiLocations;
    std::map<int, int> wifiId2Index;

    std::string filename = dirPath + "/wifi.map";
    std::ifstream wifiFile(filename.c_str());
    if (!wifiFile.is_open()) {
        std::cout << "[DataReadWrite::readMap] Error! Could not open " << (dirPath + "/wifi.map").c_str() << " file"
                  << std::endl;
    }

    int index = 0;
    while (!wifiFile.eof() && !wifiFile.fail()) {
        LocationWiFi curLoc;

        uint64_t timestamp = 0;
        int locId = 0;
        int nscans = 0;
        wifiFile >> timestamp >> locId >> nscans;

//        curLoc.locationXY.id = locId;

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
                wifiFile >> level >> freq >> localTimestamp;
                if (!wifiFile.fail()) {
                    curLoc.wifiScans.emplace_back(bssid, ssid, level, freq, localTimestamp);
                }
            }

            wifiLocations.push_back(curLoc);
            wifiId2Index[locId] = index++;
        }
    }
    std::cout << "[DataReadWrite::readMap] Read " << wifiLocations.size() << " wifi locations" << std::endl;

    // Reading images
    imageLocations.clear();
    std::map<int, int> imageId2Index;

    filename = dirPath + "/imgs/images.map";
    std::ifstream imageStream(filename.c_str());
    if (!imageStream.is_open()) {
        std::cout << "[DataReadWrite::readMap] Error! Could not open " << (dirPath + "/imgs/images.map").c_str() << " file"
                  << std::endl;
    }

    index = 0;
    while (!imageStream.eof() && !imageStream.fail()) {
        LocationImage curLoc;

        uint64_t timestamp = 0;
        int locId = 0;
        long int segmentId = 0;
        std::string imageFile = "";
        imageStream >> timestamp >> locId >> imageFile >> segmentId;

        curLoc.timestamp = timestamp;
        curLoc.segmentId = segmentId;
        if (!imageStream.fail()) {
//            std::cout << "Reading " << dirPath + "/imgs/" + imageFile << std::endl;
           cv::Mat image = cv::imread(dirPath + "/imgs/" + imageFile);

           if (!image.empty())
           {
               curLoc.image = image;
           }

            imageLocations.push_back(curLoc);
            imageId2Index[locId] = index++;
        }
    }
    std::cout << "[DataReadWrite::readMap] Read " << imageLocations.size() << " image locations" << std::endl;

    // Reading positions
    std::ifstream positionsFile((dirPath + "/positions.map").c_str());
    if (!positionsFile.is_open()) {
        std::cout << "Error! Could not open " << (dirPath + "/positions.map").c_str() << " file" << std::endl;
    }
    while (!positionsFile.eof() && !positionsFile.fail()) {
        int id;
        double x, y;
        positionsFile >> id >> x >> y;
        if (!positionsFile.fail()) {

            // Is it a WiFi location?
            auto it = wifiId2Index.find(id);
            if (it != wifiId2Index.end()) {
//                std::cout << "Compare WiFi indices: " << it->first << " " << it->second << std::endl;
                int index = it->second;
                wifiLocations[index].locationXY = LocationXY(x, y, MAP_ID_INCREMENT + it->first);
            }

            // Is it an image location?
            auto it2 = imageId2Index.find(id);
            if (it2 != imageId2Index.end()) {
//                std::cout << "Compare image indices: " << it2->first << " " << it2->second << std::endl;
                int index = it2->second;
                imageLocations[index].locationXY = LocationXY(x, y, MAP_ID_INCREMENT + it2->first);
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

    return wifiLocations;
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

std::vector<Wall> DataReadWrite::readWalls(const std::string &dirPath) {

    std::cout << "[DataReadWrite::readWalls] Reading wall data" << std::endl;

    std::ifstream wallFile((dirPath + "/walls.map").c_str());
    if (!wallFile.is_open()) {
        std::cout << "Error! Could not open " << dirPath + "/walls.map" <<
                  std::endl;
    }

    std::vector< std::vector< uint64_t > > nodesNeighbours;
    std::vector< std::pair<double, double> > nodes;

    while (!wallFile.eof() & !wallFile.fail()) {

        uint64_t numberOfNodes;
        wallFile >> numberOfNodes;

        if (!wallFile.fail()) {

            // Number of nodes for the walls
            for (int i=0;i<numberOfNodes;i++) {

                // Metric position of a node
                std::pair<double, double> position;
                wallFile >> position.first >> position.second;
                nodes.emplace_back(position);

                // Neighbours
                int numberOfNeighbours;
                wallFile >> numberOfNeighbours;
//                std::cout << "numberOfNeighbours: " << numberOfNeighbours << std::endl;

                std::vector< uint64_t > neighbours(numberOfNeighbours,0);
                for (int j=0;j<numberOfNeighbours;j++) {
                    wallFile >> neighbours[j];
                }

                nodesNeighbours.push_back(neighbours);
            }
        }
    }

    std::cout << "[DataReadWrite::readOrient] Wall data successfully read - walls: " << nodes.size() << " " << nodesNeighbours.size() << std::endl;

    // Putting it in a format that is easier to process
    std::vector<Wall> walls;
    for (int i=0;i<nodes.size();i++) {

        Wall wall;
        wall.startX = nodes[i].first;
        wall.startY = nodes[i].second;

        for (int j=0;j<nodesNeighbours[i].size();j++) {
            int secondIdx = nodesNeighbours[i][j];

            wall.endX = nodes[secondIdx].first;
            wall.endY = nodes[secondIdx].second;

//            std::cout << wall.startX << " " << wall.startY<< " " << wall.endX << " " << wall.endY << std::endl;

            walls.push_back(wall);
        }
    }

    return walls;
}


void DataReadWrite::sparsifyMapPercent(std::vector<LocationWiFi> & wifiMap, double keepPercent) {

    // We leave only keepPercent of original measurements
    int wantedSize = keepPercent * wifiMap.size();

    // Shuffle
    std::shuffle(wifiMap.begin(), wifiMap.end(), std::mt19937{std::random_device{}()});

    // Leave only some elements
    wifiMap.resize(wantedSize);

}

std::pair<double, cv::Mat> DataReadWrite::readBuildingPlan(const std::string &dirPath) {
    std::pair<double, cv::Mat> result;

    std::cout << "[DataReadWrite::readMap] Reading map" << std::endl;

    std::ifstream scaleFile((dirPath + "/scale.map").c_str());
    if (!scaleFile.is_open()) {
        std::cout << "Error! Could not open " << dirPath + "/scale.map" <<
                  std::endl;
    }

    scaleFile >> result.first;
    result.second = cv::imread(dirPath + "/map.png");

    return result;
}