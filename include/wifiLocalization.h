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
#ifndef GRAPHLOCALIZATION_WIFILOCALIZATION_H
#define GRAPHLOCALIZATION_WIFILOCALIZATION_H

#include <string>
#include <vector>
#include "parameters.h"

struct LocationXY {
    LocationXY() {}

    LocationXY(double ix, double iy, int id) : x(ix), y(iy), id(id) {}

    int id;
    double x, y;
};

struct ScanResult{
    ScanResult() {}

    ScanResult(const std::string &bssid,
               const std::string &ssid,
               double level,
               int freq,
               int localTimestamp)
            : bssid(bssid), ssid(ssid), level(level), freq(freq), localTimestamp(localTimestamp) {}

    std::string bssid, ssid;
    double level;
    int freq;
    uint64_t localTimestamp;
};

class LocationWiFi {
public:
    LocationWiFi() {}

    uint64_t timestamp;
    LocationXY locationXY;
    std::vector<ScanResult> wifiScans;
private:

};

std::vector<std::pair<double, int>> wknnWeights(const std::vector<LocationWiFi> &database,
                                                const LocationWiFi &scan,
                                                int k);


#endif //GRAPHLOCALIZATION_WIFILOCALIZATION_H
