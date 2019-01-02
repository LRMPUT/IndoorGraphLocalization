/*
 * This file is part of the IndoorGraphLocalization distribution (https://github.com/LRMPUT/IndoorGraphLocalization).
 * Copyright (c) 2018 Jan Wietrzykowski & Michał Nowicki (michal.nowicki@put.poznan.pl)
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

#ifndef GRAPHLOCALIZATION_STEPOMETER_H
#define GRAPHLOCALIZATION_STEPOMETER_H

#include <vector>

class Stepometer {
public:
    static double computeDist(const std::vector<double> &accMagSamples,
                              double dt,
                              double accSampFreq,
                              double stepLen,
                              double freqMin,
                              double freqMax,
                              double fftMagThresh);
};

#endif //GRAPHLOCALIZATION_STEPOMETER_H
