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


struct ImageRecognitionResult {

    int correctRecognitions;
    std::vector<double> matchingWeights;
    std::vector<LocationXY> matchingLocations;
};


class FastABLE {


public:
    /**
     * Initializes the visual place recognition
     * @param patchSize
     * @param compareLength
     */
    FastABLE(double patchSize, int compareLength);

    /**
     * Adds image map for recognition and computes the thresholds
     * @param imageMap
     */
    void addImageMap(const std::vector<LocationImage> &imageMap);

    /**
     * Adds new image for recognition
     * @param image
     */
    LocationXY addNewTestingImage(cv::Mat image);


private:

    /*
     * Global_description from:
     *  * @file    OpenABLE.cpp
        * @brief   Core functions of the open place recognition method called ABLE
            (Able for Binary-appearance Loop-closure Evaluation)
        * @author  Roberto Arroyo
     */
    cv::Mat global_description(cv::Mat imageIn);

    /*
     * Computing the hamming distance between two descriptors
     */
    unsigned long long hamming_matching(cv::Mat desc1, cv::Mat desc2);

    /**
     * Computes the hamming distances between provided window and selected map sequence
     * @param imageCounter
     * @param testDescriptors
     * @param trainigDescriptors
     * @param previousDistances
     * @param onePriorToWindow
     * @return
     */
    std::vector<unsigned long long> matchWindowToSingleSequence(int imageCounter, const std::vector<cv::Mat> &trainingDescriptors,
                                                      const std::vector<cv::Mat> &testDescriptors,
                                                      cv::Mat onePriorToWindow,
                                                      std::vector<unsigned long long> &previousDistance);

    /**
     *
     * @param trainingDescriptors
     * @param testDescriptorsWindow
     * @param onePriorToWindow
     * @param previousDistances
     * @return
     */
    std::vector<std::vector<unsigned long long>> matchWindowToSequences(const std::vector<std::vector<cv::Mat> > &trainingDescriptors,
                                                                        const std::vector<cv::Mat> &testDescriptorsWindow,
                                                                        const cv::Mat onePriorToWindow,
                                                                        std::vector<std::vector<unsigned long long> > &previousDistances);


    ImageRecognitionResult performRecognition(const std::vector<cv::Mat> &testDescriptorsWindow, const cv::Mat onePriorToWindow);

    /**
     *
     * @param trainingDescriptors
     * @return
     */
    std::vector<unsigned long long> automaticThresholdEstimation(const std::vector<std::vector<Mat> > &trainingDescriptors);

    /**
     *
     * @param trainingDescriptors
     * @param testDescriptors
     * @return
     */
    unsigned long long determineMinimalHammingDistance(const std::vector<std::vector<cv::Mat> > &trainingDescriptors,
                                                                 const std::vector<cv::Mat> &testDescriptors);


    LocationXY bestLocationGuess(ImageRecognitionResult imageRecognitionResult);


    // Parameters
    double patchSize;
    int compareLength;

    // Map stored as a series of segments of patches
    std::vector<std::vector<cv::Mat>> mapImageSegments;
    std::vector<std::vector<LocationXY>> mapImageSegmentLocations;

    // Corresponding thresholds for segments
    std::vector<unsigned long long> mapImageSegThresholds;

    // Vector of previous distances
    std::vector<std::vector<unsigned long long> > previousDistances;

    // Current id of processed images
    int imageCounter;

    // Accumulated descriptors for lastimages
    std::vector<cv::Mat> imgDescWindow;
    cv::Mat lastImageNotInWindow;


};


#endif //GRAPHLOCALIZATION_FASTABLE_H
