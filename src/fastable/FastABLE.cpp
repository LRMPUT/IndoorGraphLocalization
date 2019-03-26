//
// Created by mnowicki on 24.03.19.
//

#include "fastable/FastABLE.h"

FastABLE::FastABLE(double patchSize, int compareLength) : patchSize(patchSize), compareLength(compareLength) {


}


void FastABLE::addImageMap(const std::vector<LocationImage> &imageMap) {

    std::cout << "FastABLE - to vector of vector!" << std::endl;

    std::vector<cv::Mat> imageSegment;
    for (int i = 0; i < imageMap.size(); i++) {


        // Different segment
        if (!imageSegment.empty() && imageMap[i - 1].segmentId != imageMap[i].segmentId) {
            mapImageSegments.push_back(imageSegment);
            imageSegment.clear();
        }

        imageSegment.push_back(global_description(imageMap[i].image));

    }
    mapImageSegments.push_back(imageSegment);

    std::cout << "FastABLE - automaticThresholdEstimation for " << mapImageSegments.size() << " segments" << std::endl;
    for (auto &seg : mapImageSegments) {
        std::cout << "FastABLE - segment size : " << seg.size() << std::endl;
    }

    std::vector<double> thresholds = automaticThresholdEstimation(mapImageSegments);

    for (double &v : thresholds)
        std::cout << "FastABLE estimated thresholds: " << v << std::endl;


}


void FastABLE::addNewTestingImage(cv::Mat image) {

    // Computing descriptor
    cv::Mat imgDesc = global_description(image);
    imgDescWindow.push_back(imgDesc);

    if (imgDescWindow.size() > compareLength)
        imgDescWindow.erase(imgDescWindow.begin());


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

int FastABLE::hamming_matching(cv::Mat desc1, cv::Mat desc2) {

    int distance = 0;

    if (desc1.rows != desc2.rows || desc1.cols != desc2.cols || desc1.rows != 1
        || desc2.rows != 1) {

        cout << "The dimension of the descriptors is different." << desc1.rows << " " << desc1.cols << " " << desc2.rows
             << " " << desc2.cols << endl;
        return -1;

    }

    for (int i = 0; i < desc1.cols; i++) {
        distance += __builtin_popcount((*(desc1.ptr<unsigned char>(0) + i))
                                       ^ (*(desc2.ptr<unsigned char>(0) + i)));
    }

    // MNowicki: Alternatively, it is possible to compute distance with OpenCV
    //double dist = cv::norm( desc1, desc2, NORM_HAMMING);

    return distance;
}

void FastABLE::fastABLE_matching(int imageCounter, const std::vector<cv::Mat> &testDescriptors,
                                 const std::vector<cv::Mat> &trainigDescriptors,
                                 std::vector<int> &previousDistances, std::vector<int> &matchingResult) {


    int trainingSize = trainigDescriptors.size();

    matchingResult = std::vector<int>(trainingSize, std::numeric_limits<int>::max());

    if (imageCounter >= compareLength - 1) {

        for (int trainingShift = trainingSize - 1, iter = 0;
             trainingShift >= compareLength - 1;
             trainingShift--, iter++) {

            float distance = 0.0;

            // If it is not the first matching (we create results in else) and to make sure we check that those previousDistances exist
            if (imageCounter != compareLength - 1
                && trainingShift != compareLength - 1
                && previousDistances.empty() == false) {
//                std::cout << "FAST - start" << std::endl;

                // We compute current result as a result from previous iteration adding last and subtracting first
                distance = previousDistances[iter + 1]
                           + hamming_matching(testDescriptors[imageCounter],
                                              trainigDescriptors[trainingShift])
                           - hamming_matching(
                        testDescriptors[imageCounter - compareLength + 1],
                        trainigDescriptors[trainingShift - compareLength + 1]);
//                distance = previousDistances[iter + 1]
//                           + hamming_matching(testDescriptors.back(),
//                                              trainigDescriptors[trainingShift])
//                           - hamming_matching(
//                        testDescriptors.front(),
//                        trainigDescriptors[trainingShift - compareLength + 1]);

                // Let's save those precomputed distances for next iteration
                previousDistances[iter] = distance;

//                std::cout << "FAST - start" << std::endl;
            }
                // We need to compute it normally for first iteration and for one matching in each following iteration
            else {
//                std::cout << "Open - start" << std::endl;
                // for each element in the sequence to be matched
                for (int k = 0; k < compareLength; k++) {
                    // Distance between:
                    // 		testDescriptors shifted by (imageCounter-k)
                    // 		trainDescriptors shifted by (trainingShift-k)
                    distance = distance
                               + hamming_matching(testDescriptors[imageCounter - k],
                                                  trainigDescriptors[trainingShift - k]);

//                    std::cout << "Crash -> " << compareLength - k - 1 << std::endl;
//                    distance = distance
//                               + hamming_matching(testDescriptors[compareLength - k - 1],
//                                                  trainigDescriptors[trainingShift - k]);
                }
                // if we just started, we initialize the previouseDistances to later use modified recurrence
                if (imageCounter == compareLength - 1)
                    previousDistances.push_back(distance);

                    // the last one in series, so we update the previouseDistances at [0]
                else if (trainingShift == compareLength - 1)
                    previousDistances[iter] = distance;

//                std::cout << "Open - end" << std::endl;
            }

            // Saving the result
            matchingResult[trainingShift - compareLength + 1] = distance;
        }
    }
}

int FastABLE::computeVisualPlaceRecognition(const std::vector<std::vector<cv::Mat> > &trainingDescriptors,
                                            const std::vector<cv::Mat> &testDescriptors) {

    // Used to save previous results
    std::vector<std::vector<int> > previousDistances{
            trainingDescriptors.size(), std::vector<int>()};

    //
    int globalMin = std::numeric_limits<int>::max();

    // For each new test descriptor
    for (int imageCounter = 0; imageCounter < testDescriptors.size(); imageCounter++) {

        int indexBegin = std::max(0, imageCounter - compareLength + 1);
        int indexEnd = imageCounter;
        std::vector<cv::Mat> testDescriptorsWindow(testDescriptors.begin() + indexBegin,
                                                   testDescriptors.begin() + indexEnd);

        // For each map segment
        for (uint i = 0; i < trainingDescriptors.size(); i++) {

            std::vector<int> matchingResult;

            // Image matching - new version to compare time
            fastABLE_matching(imageCounter, testDescriptors, //testDescriptorsWindow,
                              trainingDescriptors[i], previousDistances[i], matchingResult);

            // Current minimum
            int localMin = *std::min_element(matchingResult.begin(), matchingResult.end());
            globalMin = std::min(globalMin, localMin);

        }
    }

    return globalMin;
}

/*
 * Code from FastABLE to find proper thresholds
 */
std::vector<double> FastABLE::automaticThresholdEstimation(const std::vector<std::vector<Mat> > &trainingDescriptors) {
    std::cout << "FastABLE - automaticThresholdEstimation" << std::endl;

    std::vector<double> localThresholds;
    double globalMin = std::numeric_limits<double>::max();
    for (uint i = 0; i < trainingDescriptors.size(); i++) {

        // Selecting one training sequence as test
        std::vector<Mat> chosenDesc = trainingDescriptors[i];

        // The remaining sequences remain as training sequences
        std::vector<std::vector<Mat> > tmpGroundTruthDescriptors(
                trainingDescriptors);
        tmpGroundTruthDescriptors.erase(tmpGroundTruthDescriptors.begin() + i);

        std::cout << "FastABLE - computeVisualPlaceRecognition" << std::endl;
        int minThreshold = computeVisualPlaceRecognition(tmpGroundTruthDescriptors, chosenDesc); // similarityMatrices

        std::cout << "FastABLE - minimal threshold = " << minThreshold * 1.0 / compareLength << std::endl;
    }

    return localThresholds;
}