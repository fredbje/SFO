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

#include "libviso2/matrix.h"
#include "libviso2/viso_stereo.h"

//#include "GeographicLib/Config.h"

#include "stereo.h"
#include "drawer.h"

namespace bfs = boost::filesystem;

void loadImages(const bfs::path &sequenceDirPath, std::vector<bfs::path> &leftImageFileNames,
                std::vector<bfs::path> &rightImageFileNames);
void loadTimeStamps(const bfs::path &timeStampsFileName, std::vector<double> &timeStamps);
void loadGtPoses(std::string gtPosesFileName);

int main(int argc, char **argv){
    if(argc != 4)
    {
        std::cerr << std::endl << "Usage: ./SFO_main path_to_sequence path_to_timestamps path_to_gt_poses" << std::endl;
        return 1;
    }

    std::vector<bfs::path> leftImageFileNames, rightImageFileNames;
    loadImages(argv[1], leftImageFileNames, rightImageFileNames);

    std::vector<double> timeStamps;
    loadTimeStamps(argv[2], timeStamps);

    loadGtPoses(argv[3]);

    // calibration parameters for odometery sequence 00
    libviso2::VisualOdometryStereo::parameters param;
    param.calib.f  = 718.85; // focal length in pixels
    param.calib.cu = 607.19; // principal point (u-coordinate) in pixels
    param.calib.cv = 185.21; // principal point (v-coordinate) in pixels
    param.base     = 0.5371; // baseline in meters

    //setup the camera calibration
    double fx, fy, s, px, py, b;
    fx = param.calib.f;
    fy = param.calib.f;
    s = 0;
    px = param.calib.cu;
    py = param.calib.cv;
    b = param.base;

    // construct the stereo calibration shared pointer, no need to delete it
    const gtsam::Cal3_S2Stereo::shared_ptr K(new gtsam::Cal3_S2Stereo(fx, fy, s, px, py, b));

    double sigmaPixel = 2;
    const gtsam::noiseModel::Isotropic::shared_ptr model = gtsam::noiseModel::Isotropic::Sigma(3, sigmaPixel);

    // init visual odometry
    auto *viso = new libviso2::VisualOdometryStereo(param);

    std::vector<int32_t> inliers;
    // Vector of structs p_match for storing matches.
    std::vector<libviso2::Matcher::p_match> pM;

    //// For display the pose, begin the thread
    // init the drawer
    SFO::Drawer poseDrawer(argv[3]);
    // Start the drawer thread
    boost::thread threadDrawer(boost::bind(&SFO::Drawer::start, poseDrawer));
/////////////////////////////////////////////

    // current pose (this matrix transforms a point from the current
    // frame's camera coordinates to the first frame's camera coordinates)
    libviso2::Matrix gtsamPose = libviso2::Matrix::eye(4);
    libviso2::Matrix libviso2Pose = libviso2::Matrix::eye(4);

    ////// the third pointer
    auto *gtsamPoseVec = new std::vector<libviso2::Matrix>();
    auto *libviso2PoseVec = new std::vector<libviso2::Matrix>();

    gtsamPoseVec->push_back(gtsamPose);
    libviso2PoseVec->push_back(libviso2Pose);

    poseDrawer.updatePoses(gtsamPoseVec);
    poseDrawer.updatePoseslib(libviso2PoseVec);

    // Allocate variables for the loop
    cv::Size refImageSize = cv::imread(leftImageFileNames[0].string()).size();
    cv::Mat img_C1 = cv::Mat(refImageSize, CV_8UC1);
    cv::Mat img_C2 = cv::Mat(refImageSize, CV_8UC1);
    std::int32_t dims[] = {refImageSize.width, refImageSize.height, refImageSize.width};

    //                          4541
    for (std::size_t i = 0; i < 4541; i++) {

        try {
            img_C1 = cv::imread(leftImageFileNames[i].string(), cv::IMREAD_GRAYSCALE);
            img_C2 = cv::imread(rightImageFileNames[i].string(), cv::IMREAD_GRAYSCALE);

            if(img_C1.size() != refImageSize || img_C2.size() != refImageSize) {
                std::cerr << "Images in sequence have different size." << std::endl;
                break;
            }

            std::cout << "Processing: Frame: " << std::setw(4) << i;

            if (viso->process(img_C1.data, img_C2.data, dims)) {
                // on success, update current pose
                // get the matches
                pM.clear();
                inliers.clear();
                pM = viso->getMatches();
                inliers = viso->getInlierIndices();

                // initial values choices:
                //libviso2::Matrix poseInit = libviso2::Matrix::inv(viso->getMotion());
                libviso2::Matrix poseInit = libviso2::Matrix::eye(4);

                libviso2::Matrix poseOpt(4, 4);

                //TODO: Local Optimization to get an optimized relative pose and feature point
                SFO::localOptimization(pM, inliers, poseInit, param, sigmaPixel, K, model, poseOpt);

                gtsamPose = gtsamPose * (poseOpt);
                gtsamPoseVec->push_back(gtsamPose);

                libviso2Pose = libviso2Pose * libviso2::Matrix::inv(viso->getMotion());
                libviso2PoseVec->push_back(libviso2Pose);

                // output some statistics
                double num_matches = viso->getNumberOfMatches();
                double num_inliers = viso->getNumberOfInliers();
                std::cout << ", Matches: " << std::fixed << std::setprecision(0) << num_matches;
                std::cout << ", Inliers: " << std::fixed << std::setprecision(2)
                          << 100.0 * num_inliers / num_matches << " %" << std::endl;

                poseDrawer.updatePoses(gtsamPoseVec);
                poseDrawer.updatePoseslib(libviso2PoseVec);

                // Create combined matrix
                cv::Mat im3(2*refImageSize.height + 5, refImageSize.width, CV_8UC1);
                cv::Mat left(im3, cv::Rect(0, 0, refImageSize.width, refImageSize.height));
                img_C1.copyTo(left);
                cv::Mat right(im3, cv::Rect(0, refImageSize.height + 5, refImageSize.width, refImageSize.height));
                img_C2.copyTo(right);

                // Convert to color type
                cv::cvtColor(im3, im3, CV_GRAY2RGB);

                // ==============================
                // Add extracted features to mat
                // ==============================
                for (std::size_t ki = 0; ki < pM.size(); ki++) {
                    // Get match data
                    libviso2::Matcher::p_match match;
                    match = pM.at(ki);
                    // Point color object
                    cv::Scalar color;
                    // Check to see if inlier, if so color it as one
                    // The outliers are red
                    if (std::find(inliers.begin(), inliers.end(), ki) != inliers.end()) {
                        color = CV_RGB(0, 255, 255);
                    } else {
                        color = CV_RGB(255, 0, 0);
                    }
                    // Add to image
                    cv::Point2f pt_left(match.u1c, match.v1c);
                    cv::circle(im3, pt_left, 2, color);
                    cv::Point2f pt_right(match.u2c, match.v2c + (refImageSize.height + 5));
                    cv::circle(im3, pt_right, 2, color);
                }

                // Display it
                cv::imshow("Stereo Gray Image", im3);
                cv::waitKey(10);

            } else if(i == 0) {
                std::cout << std::endl;
            } else {
                std::cerr << " ... failed!" << std::endl;
            }

            // catch image read errors here
        } catch (...) {
            std::cerr << "ERROR: Couldn't read input files!" << std::endl;
            return 1;
        }
    } //end for


    // Stop the visualization thread if it was running
    // https://www.quantnet.com/threads/c-multithreading-in-boost.10028/
    while (!threadDrawer.timed_join(boost::posix_time::seconds(1))) {
        threadDrawer.interrupt();
    }

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













