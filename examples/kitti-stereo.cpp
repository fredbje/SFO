#include<iostream>
#include<iomanip>
#include<thread>

//#include<Eigen/Core>

//#include <boost/lambda/lambda.hpp>
//#include <boost/thread.hpp>
#include <boost/filesystem.hpp>

// #include <png++/png.hpp>

//#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
//#include <opencv2/imgcodecs.hpp>
//#include <opencv2/imgproc.hpp>

#include "libviso2/matrix.h"
#include "libviso2/viso_stereo.h"

//#include "GeographicLib/Config.h"

#include "stereo.h"
#include "mapDrawer.h"
#include "frameDrawer.h"

void loadImageFileNames(const std::string &strSequenceDir, std::vector<std::string> &vstrLeftImages,
                std::vector<std::string> &vstrRightImages);
void readImages(cv::Mat &imgLeft, cv::Mat &imgRight, const std::string &strLeftImage,
                const std::string &strRightImage, const cv::Size &szRefSize);
void loadTimeStamps(const std::string &strTimestampsFile, std::vector<double> &vTimestamps);
void loadGtPoses(const std::string &strGtPosesFile, std::vector<libviso2::Matrix> &vGtPoses);

int main(int argc, char **argv){
    if(argc != 5) {
        std::cerr << std::endl << "Usage: ./SFO_main path_to_sequence path_to_settings_file "
                "path_to_timestamps path_to_gt_poses" << std::endl;
        return 1;
    }

    std::string strSequenceDir = argv[1];
    std::string strSettingsFile = argv[2];
    std::string strTimestampsFile = argv[3];
    std::string strGtPosesFile = argv[4];

    std::vector<std::string> vstrLeftImages;
    std::vector<std::string> vstrRightImages;
    loadImageFileNames(strSequenceDir, vstrLeftImages, vstrRightImages);

    std::vector<double> vTimestamps;
    loadTimeStamps(strTimestampsFile, vTimestamps);

    if(vTimestamps.size() != vstrLeftImages.size()) {
        std::cerr << "Number of timestamps does not match number of images" << std::endl;
        return 1;
    }

    std::vector<libviso2::Matrix> vGtPoses;
    loadGtPoses(strGtPosesFile, vGtPoses);

    if(vGtPoses.size() != vstrLeftImages.size()) {
        std::cerr << "Number of GT poses does not match number of images." << std::endl;
        return 1;
    }

    cv::FileStorage fSettings(strSettingsFile, cv::FileStorage::READ);

    if(!fSettings.isOpened()) {
        std::cerr << "Failed to open settings file at: " << strSettingsFile << std::endl;
    }

    float fx = fSettings["Camera.fx"]; // focal length in pixels
    float fy = fSettings["Camera.fy"]; // focal length in pixels
    float cx = fSettings["Camera.cx"]; // principal point (x/u-coordinate) in pixels
    float cy = fSettings["Camera.cy"]; // principal point (y/v-coordinate) in pixels
    float bf = fSettings["Camera.bf"];
    float base = bf/fx;                // baseline in meters
    float s = 0;                       // shear

    cv::Size szImgSize(fSettings["Camera.width"], fSettings["Camera.height"]);

    float fps = fSettings["Camera.fps"];
    double T = (fps < 1) ? 1/30 : 1/fps;

    fSettings.release();

    libviso2::VisualOdometryStereo::parameters param;
    param.calib.f  = fx;
    param.calib.cu = cx;
    param.calib.cv = cy;
    param.base     = base;

    auto *pViso = new libviso2::VisualOdometryStereo(param);

    // construct the stereo calibration shared pointer, no need to delete it
    const gtsam::Cal3_S2Stereo::shared_ptr K(new gtsam::Cal3_S2Stereo(fx, fy, s, cx, cy, base));
    double sigmaPixel = 2;
    const gtsam::noiseModel::Isotropic::shared_ptr model = gtsam::noiseModel::Isotropic::Sigma(3, sigmaPixel);

    auto *pDrawer = new SFO::MapDrawer(strSettingsFile, vGtPoses);
    std::thread tDrawer(&SFO::MapDrawer::start, pDrawer);

/////////////////////////////////////////////

    // current pose (this matrix transforms a point from the current
    // frame's camera coordinates to the first frame's camera coordinates)
    libviso2::Matrix gtsamPose = libviso2::Matrix::eye(4);
    libviso2::Matrix libviso2Pose = libviso2::Matrix::eye(4);

    auto *pvGtsamPoses = new std::vector<libviso2::Matrix>();
    auto *pvLibviso2Poses = new std::vector<libviso2::Matrix>();

    pvGtsamPoses->push_back(gtsamPose);
    pvLibviso2Poses->push_back(libviso2Pose);

    pDrawer->updateGtsamPoses(pvGtsamPoses);
    pDrawer->updateLibviso2Poses(pvLibviso2Poses);

    cv::Mat imgLeft(szImgSize, CV_8UC1);
    cv::Mat imgRight(szImgSize, CV_8UC1);
    SFO::FrameDrawer frameDrawer(pViso, szImgSize);

    std::vector<int32_t> vInliers;
    std::vector<libviso2::Matcher::p_match> vMatches;
    std::int32_t dims[] = {static_cast<std::int32_t>(szImgSize.width),
                           static_cast<std::int32_t>(szImgSize.height),
                           static_cast<std::int32_t>(szImgSize.width)};
    for (std::size_t i = 0; i < 50/*vstrLeftImages.size()*/; i++) { // 4541 images

        readImages(imgLeft, imgRight, vstrLeftImages[i], vstrRightImages[i], szImgSize);

        std::cout << "Processing: Frame: " << std::setw(4) << i;

        if (pViso->process(imgLeft.data, imgRight.data, dims)) {
            vMatches.clear();
            vInliers.clear();
            vMatches = pViso->getMatches();
            vInliers = pViso->getInlierIndices();

            //libviso2::Matrix poseInit = libviso2::Matrix::inv(viso->getMotion());

            libviso2::Matrix poseInit = libviso2::Matrix::eye(4); // Initial guess?
            libviso2::Matrix poseOpt(4, 4);

            //TODO: Local Optimization to get an optimized relative pose and feature point
            SFO::localOptimization(vMatches, vInliers, poseInit, param, sigmaPixel, K, model, poseOpt);

            gtsamPose = gtsamPose * (poseOpt);
            pvGtsamPoses->push_back(gtsamPose);

            libviso2Pose = libviso2Pose * libviso2::Matrix::inv(pViso->getMotion());
            pvLibviso2Poses->push_back(libviso2Pose);

            pDrawer->updateGtsamPoses(pvGtsamPoses);
            pDrawer->updateLibviso2Poses(pvLibviso2Poses);

            frameDrawer.update(imgLeft, imgRight);

            cv::imshow("Stereo Gray Image", frameDrawer.drawFrame());
            cv::waitKey(static_cast<int>(T*1e3));
        } else if(i != 0) {
            std::cerr << " ... failed!";
        }
        std::cout << std::endl;
    } //end for(int32_t i = 0; i < 4500; i++)

    pDrawer->requestFinish();
    tDrawer.join();

    delete pDrawer;
    delete pvGtsamPoses;
    delete pvLibviso2Poses;
    delete pViso;

    std::cout << "SFO_main complete! Exiting ..." << std::endl;

	return 0;
}

namespace bfs = boost::filesystem;

void loadImageFileNames(const std::string &strSequenceDir, std::vector<std::string> &vstrLeftImages,
                std::vector<std::string> &vstrRightImages) {
    if(!bfs::is_directory(strSequenceDir)) {
        std::cerr << strSequenceDir << " is not a directory." << std::endl;
        return;
    }
    std::cout << "Looking for images in " << strSequenceDir << std::endl;

    bfs::path pathLeftImageDir = bfs::path(strSequenceDir) / "image_0";
    bfs::path pathRightImageDir = bfs::path(strSequenceDir) / "image_1";

    if(!bfs::is_directory(pathLeftImageDir)) {
        std::cerr << pathLeftImageDir << " is not a directory" << std::endl;
        return;
    } else if(!bfs::is_directory(pathRightImageDir)) {
        std::cerr << pathRightImageDir << " is not a directory" << std::endl;
        return;
    }

    for(auto& direntryFile : bfs::directory_iterator(pathLeftImageDir)) {
        if(direntryFile.path().extension() == ".png") {
            vstrLeftImages.emplace_back(direntryFile.path().string());
        } else {
            std::cerr << direntryFile << " is not a png image" << std::endl;
        }
    }

    for(auto& direntryFile : bfs::directory_iterator(pathRightImageDir)) {
        if(direntryFile.path().extension() == ".png") {
            vstrRightImages.emplace_back(direntryFile.path().string());
        } else {
            std::cerr << direntryFile << " is not a png image" << std::endl;
        }
    }

    if(vstrLeftImages.size() != vstrRightImages.size()) {
        std::cerr << "The left and right image directories contains unequal amounts of images." << std::endl;
    }

    std::sort(vstrLeftImages.begin(), vstrLeftImages.end());
    std::sort(vstrRightImages.begin(), vstrRightImages.end());

}

void readImages(cv::Mat &imgLeft, cv::Mat &imgRight, const std::string &strLeftImage,
                const std::string &strRightImage, const cv::Size &szRefSize) {
    imgLeft = cv::imread(strLeftImage, cv::IMREAD_GRAYSCALE);
    imgRight = cv::imread(strRightImage, cv::IMREAD_GRAYSCALE);

    if (imgLeft.empty()) {
        std::cerr << "Couldn't read image from" << strLeftImage << std::endl;
    } else if(imgRight.empty()) {
        std::cerr << "Couldn't read image from" << strRightImage << std::endl;
    }

    if(imgLeft.size() != szRefSize || imgRight.size() != szRefSize) {
        std::cout << "Error, images have different size than specified in settings file." << std::endl;
    }

}

void loadTimeStamps(const std::string &strTimestampsFile, std::vector<double> &vTimestamps) {

    if(!bfs::is_regular_file(strTimestampsFile)) {
        std::cerr << strTimestampsFile << " is not a file." << std::endl;
        return;
    }

    std::ifstream fTimes;
    fTimes.open(strTimestampsFile.c_str());

    while(!fTimes.eof()) {
        std::string s;
        getline(fTimes,s);
        if(!s.empty()) {
            std::stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }
}

void loadGtPoses(const std::string &strGtPosesFile, std::vector<libviso2::Matrix> &vGtPoses) {
    if(!bfs::is_regular_file(bfs::path(strGtPosesFile))) {
        std::cerr << "Could not open GT poses at: " << strGtPosesFile << std::endl;
    }

    std::cout << "Loading GT poses from: " << strGtPosesFile << std::endl;

    std::ifstream f;
    std::string strLine;

    f.open(strGtPosesFile.c_str(), std::ifstream::in);
    if(!f.is_open()){
        std::cerr << "Error, could not open ground truth pose file!" << std::endl;
    }

    while(std::getline(f, strLine)) {
        std::istringstream iss(strLine);
        double r11, r12, r13, r21, r22, r23, r31, r32, r33, t1, t2, t3;

        if(!(iss >> r11 >> r12 >> r13 >> t1 >> r21 >> r22 >> r23 >> t2 >> r31 >> r32 >> r33 >> t3)) {
            break;
        }

        libviso2::Matrix tempPose(4,4);
        tempPose.val[0][0] = r11;
        tempPose.val[0][1] = r12;
        tempPose.val[0][2] = r13;
        tempPose.val[0][3] = t1;
        tempPose.val[1][0] = r21;
        tempPose.val[1][1] = r22;
        tempPose.val[1][2] = r23;
        tempPose.val[1][3] = t2;
        tempPose.val[2][0] = r31;
        tempPose.val[2][1] = r32;
        tempPose.val[2][2] = r33;
        tempPose.val[2][3] = t3;
        tempPose.val[3][0] = 0;
        tempPose.val[3][1] = 0;
        tempPose.val[3][2] = 0;
        tempPose.val[3][3] = 1;
        vGtPoses.push_back(tempPose);
    }
    std::cout << "Finished loading GT poses." << std::endl;
}
