//
// Created by mnowicki on 24.03.19.
//

#include "fastable/FastABLE.h"

FastABLE::FastABLE(double patchSize, int compareLength) : patchSize(patchSize), compareLength(compareLength) {


}


void FastABLE::addImageMap(const std::vector<LocationImage> &imageMap) {

    // Divide into map segments
    std::vector<cv::Mat> imageSegment;
    for (int i = 0; i < imageMap.size(); i++) {

        // Different segment
        if (!imageSegment.empty() && imageMap[i - 1].segmentId != imageMap[i].segmentId) {

            // Only adding if segment is larger than compareLength
            if (imageSegment.size() > compareLength)
                mapImageSegments.push_back(imageSegment);
            imageSegment.clear();
        }

        imageSegment.push_back(global_description(imageMap[i].image));

    }


    // Only adding if segment is larger than compareLength
    if (imageSegment.size() > compareLength)
        mapImageSegments.push_back(imageSegment);

    imageCounter = 0;
    previousDistances = std::vector<std::vector<unsigned long long> >{mapImageSegments.size(), std::vector<unsigned long long>()};

    // Threshold estimation for segments
    mapImageSegThresholds = automaticThresholdEstimation(mapImageSegments);

    for (unsigned long long &v : mapImageSegThresholds)
        std::cout << "FastABLE estimated thresholds: " << v << std::endl;
}


void FastABLE::addNewTestingImage(cv::Mat image) {

    // Computing descriptor
    cv::Mat imgDesc = global_description(image);
    imgDescWindow.push_back(imgDesc);


    // Moving window of images, also saving the last one not in the window
    if (imgDescWindow.size() > compareLength) {

        lastImageNotInWindow = imgDescWindow.front();
        imgDescWindow.erase(imgDescWindow.begin());
    }

    // TODO Recognition
    if (imgDescWindow.size() == compareLength) {
        performRecognition(imgDescWindow, lastImageNotInWindow);
    }

    imageCounter++;
}


/*
 * Global_description from:
 *  * @file    OpenABLE.cpp
    * @brief   Core functions of the open place recognition method called ABLE
        (Able for Binary-appearance Loop-closure Evaluation)
    * @author  Roberto Arroyo
 */
cv::Mat FastABLE::global_description(cv::Mat imageIn) {

    // To grayscale
    cv::Mat image;
    cv::cvtColor(imageIn, image, COLOR_BGR2GRAY);

    // Resize the image
    cv::Mat image_resized;
    cv::resize(image, image_resized, cv::Size(patchSize, patchSize), 0, 0,
               INTER_LINEAR);

    // Select the central keypoint
    std::vector<cv::KeyPoint> kpts;
    cv::KeyPoint kpt;
    kpt.pt.x = patchSize / 2 + 1;
    kpt.pt.y = patchSize / 2 + 1;
    kpt.size = 1.0;
    kpt.angle = 0.0;
    kpts.push_back(kpt);

    // Computing the descriptor
    cv::Mat descriptor;
    LDB ldb;
    ldb.compute(image_resized, kpts, descriptor);

    return descriptor;

}

unsigned long long FastABLE::hamming_matching(cv::Mat desc1, cv::Mat desc2) {

    unsigned long long distance = 0;

    if (desc1.rows != desc2.rows || desc1.cols != desc2.cols || desc1.rows != 1
        || desc2.rows != 1) {

        cout << "The dimension of the descriptors is different." << desc1.rows << " " << desc1.cols << " " << desc2.rows
             << " " << desc2.cols << endl;
        return -1;

    }

    for (unsigned long long i = 0; i < desc1.cols; i++) {
        distance += __builtin_popcount((*(desc1.ptr<unsigned char>(0) + i))
                                       ^ (*(desc2.ptr<unsigned char>(0) + i)));
    }

    // MNowicki: Alternatively, it is possible to compute distance with OpenCV
    //double dist = cv::norm( desc1, desc2, NORM_HAMMING);

    return distance;
}

std::vector<unsigned long long> FastABLE::matchWindowToSingleSequence(int imageCounter, const std::vector<cv::Mat> &trainingDescriptors,
                                                                      const std::vector<cv::Mat> &testDescriptorsWindow,
                                                                      cv::Mat onePriorToWindow,
                                                                      std::vector<unsigned long long> &previousDistances) {
//    std::cout << "FastABLE::matchWindowToSingleSequence - begin" << std::endl;

    std::vector<unsigned long long> matchingResult;

    int trainingSize = trainingDescriptors.size();

    std::vector<unsigned long long> nextPreviousDistances;

    for (int trainingShift = 0;
         trainingShift < trainingSize - compareLength - 1;
         trainingShift++) {


        unsigned long long distance = 0;

        // If it is possible to make it faster
        if (imageCounter != compareLength - 1 // Not first iteration
            && previousDistances.empty() == false // We have previous distances
            && trainingShift != 0 // It has to be computed normally
            ) {

            // We compute current result as a result from previous iteration adding last and subtracting first
            distance = previousDistances[trainingShift - 1]
                       + hamming_matching(testDescriptorsWindow[compareLength - 1],
                                          trainingDescriptors[trainingShift + compareLength - 1])
                       - hamming_matching(onePriorToWindow,
                                          trainingDescriptors[trainingShift - 1]);

            // Let's save those precomputed distances for next iteration
            nextPreviousDistances.push_back(distance);
        }
        // Computing everything for 1st window and for every first comparison for a new frame
        else {
            // for each element in the sequence to be matched
            for (int k = 0; k < compareLength; k++) {

                // Distance between:
                // 		testDescriptorsWindow
                // 		trainDescriptors shifted by (trainingShift)
                distance = distance
                           + hamming_matching(testDescriptorsWindow[k],
                                              trainingDescriptors[trainingShift + k]);
            }
            // if we just started, we initialize the previousDistances to later use modified recurrence
            if (imageCounter == compareLength - 1)
                nextPreviousDistances.push_back(distance);

            // the first comparison when we already have previous distances
            else if (trainingShift == 0)
                nextPreviousDistances.push_back(distance);
        }

        // Saving the result
        matchingResult.push_back(distance);
    }

    // Copying the previous results
    previousDistances = nextPreviousDistances;

//    std::cout << "FastABLE::matchWindowToSingleSequence - end" << std::endl;
    return matchingResult;
}



std::vector<std::vector<unsigned long long>> FastABLE::matchWindowToSequences(const std::vector<std::vector<cv::Mat> > &trainingDescriptors,
                                                  const std::vector<cv::Mat> &testDescriptorsWindow,
                                                  const cv::Mat onePriorToWindow,
                                                  std::vector<std::vector<unsigned long long> > &previousDistances) {
//    std::cout << "FastABLE::matchWindowToSequences - start" << std::endl;

    std::vector<std::vector<unsigned long long>> results;

    // For each map segment
    for (uint i = 0; i < trainingDescriptors.size(); i++) {

        // Image matching against i-th training part
        std::vector<unsigned long long> matchingResult = matchWindowToSingleSequence(imageCounter, trainingDescriptors[i],
                testDescriptorsWindow, onePriorToWindow, previousDistances[i]);

        results.emplace_back(matchingResult);
    }

//    std::cout << "FastABLE::matchWindowToSequences - end" << std::endl;
    return results;
}



void FastABLE::performRecognition(const std::vector<cv::Mat> &testDescriptorsWindow, const cv::Mat onePriorToWindow) {
    std::cout << "FastABLE::performRecognition - start" << std::endl;

    // Match current window
    std::vector<std::vector<unsigned long long>> resultsForAll = matchWindowToSequences( this->mapImageSegments,
            testDescriptorsWindow, onePriorToWindow, this->previousDistances);

    std::cout << "FastABLE::performRecognition - after recognition" << std::endl;

    // Go through results
    for (int i=0;i<this->mapImageSegments.size();i++) {

        // Consider matching for selected segment from map
        auto & resultForSegment = resultsForAll[i];
        auto & segmentThreshold = this->mapImageSegThresholds[i];

        // For all performed matches
        for (int j=0;j<resultForSegment.size();j++) {

            // Correct recognition
            if (resultForSegment[j] < segmentThreshold ){

                std::cout << "Correct recognition! " << resultForSegment[j] << " < " << segmentThreshold << std::endl;
                // TODO: Now determine location
            }
        }
    }

    std::cout << "FastABLE::performRecognition - end" << std::endl;
}


std::vector<unsigned long long> FastABLE::automaticThresholdEstimation(const std::vector<std::vector<Mat> > &trainingDescriptors) {
    std::cout << "FastABLE - automaticThresholdEstimation for " <<  trainingDescriptors.size() << " sequences" << std::endl;

//    for (uint i = 0; i < trainingDescriptors.size(); i++) {
//        std::cout << "Sizes : " << trainingDescriptors[i].size() << std::endl;
//    }

    // We look for separate threshold for each training sequence
    std::vector<unsigned long long> localThresholds;
    for (uint i = 0; i < trainingDescriptors.size(); i++) {

        // Selecting one training sequence as test
        std::vector<Mat> chosenDesc = trainingDescriptors[i];

        // The remaining sequences remain as training sequences
        std::vector<std::vector<Mat> > tmpGroundTruthDescriptors(trainingDescriptors);
        tmpGroundTruthDescriptors.erase(tmpGroundTruthDescriptors.begin() + i);

        // Let's compute recognitions
//        std::cout << "FastABLE - sequence id = " << i << " length = " << chosenDesc.size() << std::endl;
        unsigned long long minThreshold = determineMinimalHammingDistance(tmpGroundTruthDescriptors, chosenDesc);

        localThresholds.push_back(minThreshold * safetyThresholdRatio);

//        std::cout << "FastABLE - minimal threshold = " << minThreshold * 1.0 / compareLength << std::endl;
    }

    // Clear for new recognition
    imageCounter = 0;
    previousDistances = std::vector<std::vector<unsigned long long> >{trainingDescriptors.size(), std::vector<unsigned long long>()};

    return localThresholds;
}

unsigned long long FastABLE::determineMinimalHammingDistance(const std::vector<std::vector<cv::Mat> > &trainingDescriptors,
                                                             const std::vector<cv::Mat> &testDescriptors) {
//    std::cout << "FastABLE::determineMinimalHammingDistance - begin" << std::endl;

    // Clearing for automatic threshold
    imageCounter = 0;
    previousDistances = std::vector<std::vector<unsigned long long> >{trainingDescriptors.size(), std::vector<unsigned long long>()};

    // Let's determine the global minimum
    unsigned long long globalMin = std::numeric_limits<unsigned long long>::max();

    // For each new test descriptor
    for (int imageIndex = 0; imageIndex < testDescriptors.size(); imageIndex++) {

        // Selecting window
        int indexBegin = std::max(0, imageIndex - compareLength + 1);
        int indexLast = imageIndex;
        std::vector<cv::Mat> testDescriptorsWindow(testDescriptors.begin() + indexBegin,
                                                   testDescriptors.begin() + indexLast + 1);

        // Last frame not in a window
        cv::Mat onePriorToWindow;
        if (indexBegin > 0)
            onePriorToWindow = testDescriptors[indexBegin - 1];

//        std::cout << indexBegin << " " << indexLast << " " << testDescriptorsWindow.size() << " " << imageCounter << std::endl;

        // Proper size for comparison
        if(testDescriptorsWindow.size() == compareLength) {

            std::vector<std::vector<unsigned long long>> matchingResult = matchWindowToSequences(trainingDescriptors, testDescriptorsWindow, onePriorToWindow, previousDistances);

            // Current minimum
            for (auto &sequenceResult : matchingResult) {
//                std::cout << "? " << sequenceResult.size() << std::endl;
                unsigned long long localMin = *std::min_element(sequenceResult.begin(), sequenceResult.end());
//                std::cout << "localMin = " << localMin << " " << sequenceResult.size() <<  std::endl;

                globalMin = std::min(globalMin, localMin);
            }
        }

        imageCounter++;
    }

//    std::cout << "FastABLE::determineMinimalHammingDistance - end" << std::endl;
    return globalMin;
}
