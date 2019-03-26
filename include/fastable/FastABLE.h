//
// Created by mnowicki on 24.03.19.
//

#ifndef GRAPHLOCALIZATION_FASTABLE_H
#define GRAPHLOCALIZATION_FASTABLE_H

#include <iostream>

#include "wifiLocalization.h"
#include "ldb/ldb.h"
#include <vector>
#include <algorithm>

class FastABLE {


public:
    FastABLE(double patchSize, int compareLength);


    void addImageMap(const std::vector<LocationImage> &imageMap);


    void addNewTestingImage(cv::Mat image);


private:

    /*
     * Global_description from:
     *  * @file    OpenABLE.cpp
        * @brief   Core functions of the open place recognition method called ABLE
            (Able for Binary-appearance Loop-closure Evaluation)
        * @author  Roberto Arroyo
     */
    cv::Mat global_description(cv::Mat imageIn);

    int hamming_matching(cv::Mat desc1, cv::Mat desc2);

    void fastABLE_matching(int imageCounter, const std::vector<cv::Mat> & testDescriptors, const std::vector<cv::Mat> & trainigDescriptors,
                           std::vector<int> &previousDistances,  std::vector<int> &matchingResult);
//                           std::vector<int>& previousDistances, cv::Mat similarityMatrix);

    int computeVisualPlaceRecognition(const std::vector<std::vector<cv::Mat> >& trainingDescriptors,
            const std::vector<cv::Mat>& testDescriptors);
//            std::vector<cv::Mat>& similarityMatrices);

    /*
     * Code from FastABLE to find proper thresholds
     */
    std::vector<double> automaticThresholdEstimation(const std::vector<std::vector<Mat> >& trainingDescriptors);

    // Parameters
    double patchSize;
    int compareLength;

    // Map stored as a series of segments of patches
    std::vector<std::vector<cv::Mat>> mapImageSegments;

    // Accumulated descriptors for lastimages
    std::vector<cv::Mat> imgDescWindow;

};


#endif //GRAPHLOCALIZATION_FASTABLE_H
