#include<iostream>
#include<iomanip>
#include<algorithm>
#include<iterator>

#include<Eigen/Core>

#include <boost/lambda/lambda.hpp>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>
#include <boost/iostreams/device/back_inserter.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
//#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>



//#include "GeographicLib/Config.h"

#include "system.h"
//#include "stereo.h"
//#include "drawer.h"

namespace bfs = boost::filesystem;

void loadImages(const bfs::path &sequenceDirPath, std::vector<bfs::path> &leftImageFileNames,
                std::vector<bfs::path> &rightImageFileNames);
void loadTimeStamps(const bfs::path &timeStampsFileName, std::vector<double> &timeStamps);
void loadGtPoses(std::string gtPosesFileName);

int main(int argc, char **argv){
    if(argc != 5)
    {
        std::cerr << std::endl << "Usage: ./SFO_main path_to_sequence path_to_settings_file "
                "path_to_timestamps path_to_gt_poses" << std::endl;
        return 1;
    }

    std::vector<bfs::path> leftImageFileNames, rightImageFileNames;
    loadImages(argv[1], leftImageFileNames, rightImageFileNames);

    std::vector<double> vTimestamps;
    loadTimeStamps(argv[3], vTimestamps);

    //loadGtPoses(argv[4]);

    SFO::System SLAM(argv[2]);

    // Allocate variables for the loop
    const int nImages = leftImageFileNames.size();
    std::vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);
    cv::Size refImageSize = cv::imread(leftImageFileNames[0].string()).size();
    cv::Mat leftImg = cv::Mat(refImageSize, CV_8UC1);
    cv::Mat rightImg = cv::Mat(refImageSize, CV_8UC1);

    //                          4541
    for (std::size_t i = 0; i < leftImageFileNames.size(); i++) {

        leftImg = cv::imread(leftImageFileNames[i].string(), cv::IMREAD_GRAYSCALE);
        rightImg = cv::imread(rightImageFileNames[i].string(), cv::IMREAD_GRAYSCALE);

        if(leftImg.empty()) {
            std::cerr << "Failed to load image at: " << leftImageFileNames[i] << std::endl;
            return 1;
        } else if(rightImg.empty()) {
            std::cerr << "Failed to load image at: " << rightImageFileNames[i] << std::endl;
        }

        if(leftImg.size() != refImageSize || rightImg.size() != refImageSize) {
            std::cerr << "Images in sequence have different size." << std::endl;
            break;
        }

        std::cout << "Processing: Frame: " << std::setw(4) << i;

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        double tFrame = vTimestamps[i];

        SLAM.trackStereo(leftImg, rightImg, tFrame);

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[i]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(i<nImages-1)
            T = vTimestamps[i+1]-tFrame;
        else if(i>0)
            T = tFrame-vTimestamps[i-1];

        if(ttrack<T)
            std::this_thread::sleep_for(std::chrono::microseconds(static_cast<int>((T-ttrack)*1e6)));

    } //end for

    SLAM.shutdown();

    // Stop the visualization thread if it was running
    // https://www.quantnet.com/threads/c-multithreading-in-boost.10028/
    //while (!threadDrawer.timed_join(boost::posix_time::seconds(1))) {
     //   threadDrawer.interrupt();
    //}

    std::cout << "SFO_main complete! Exiting ..." << std::endl;


	return 0;
}

void loadImages(const bfs::path &sequenceDirPath, std::vector<bfs::path> &leftImageFileNames,
                std::vector<bfs::path> &rightImageFileNames) {
    if(!bfs::is_directory(sequenceDirPath)) {
        std::cerr << sequenceDirPath << " is not a directory." << std::endl;
        return;
    }
    std::cout << "Looking for images in " << sequenceDirPath << std::endl;

    bfs::path leftImageDirPath = sequenceDirPath / "image_0";
    bfs::path rightImageDirPath = sequenceDirPath / "image_1";

    if(!bfs::is_directory(leftImageDirPath)) {
        std::cerr << leftImageDirPath << " is not a directory" << std::endl;
        return;
    } else if(!bfs::is_directory(rightImageDirPath)) {
        std::cerr << rightImageDirPath << " is not a directory" << std::endl;
        return;
    }

    for(auto& fileName: bfs::directory_iterator(leftImageDirPath)) {
        if(fileName.path().extension() == ".png") {
            leftImageFileNames.emplace_back(fileName);
        } else {
            std::cerr << fileName << " is not an image" << std::endl;
        }
    }

    for(auto& fileName: bfs::directory_iterator(rightImageDirPath)) {
        if(fileName.path().extension() == ".png") {
            rightImageFileNames.emplace_back(fileName);
        } else {
            std::cerr << fileName << " is not an image" << std::endl;
        }
    }

    if(leftImageFileNames.size() != rightImageFileNames.size()) {
        std::cerr << "The left and right image directories contains unequal amounts of images." << std::endl;
    }

    std::sort(leftImageFileNames.begin(), leftImageFileNames.end());
    std::sort(rightImageFileNames.begin(), rightImageFileNames.end());

}

void loadTimeStamps(const bfs::path &timeStampsFileName, std::vector<double> &timeStamps) {

    if(!bfs::is_regular_file(timeStampsFileName)) {
        std::cerr << timeStampsFileName << " is not a file." << std::endl;
        return;
    }

    std::ifstream fTimes;
    fTimes.open(timeStampsFileName.c_str());

    while(!fTimes.eof()) {
        std::string s;
        getline(fTimes,s);
        if(!s.empty()) {
            std::stringstream ss;
            ss << s;
            double t;
            ss >> t;
            timeStamps.push_back(t);
        }
    }
}

void loadGtPoses(std::string gtPosesFileName) {
    std::cout << "Path to ground truth poses: " << gtPosesFileName << std::endl;
}













