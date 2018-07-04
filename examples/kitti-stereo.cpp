#include<iostream>
#include<iomanip>
#include<thread>

#include <boost/filesystem.hpp>

#include <opencv2/highgui.hpp>

#include "libviso2/matrix.h"
#include "libviso2/viso_stereo.h"

#include "gtsamTracker.h"
#include "mapDrawer.h"
#include "frameDrawer.h"
#include "system.h"

void loadImageFileNames(const std::string &strSequenceDir, std::vector<std::string> &vstrLeftImages,
                std::vector<std::string> &vstrRightImages);
void readImages(cv::Mat &imgLeft, cv::Mat &imgRight, const std::string &strLeftImage,
                const std::string &strRightImage);
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

/////////////////////////////////////////////


    SFO::System SLAM(strSettingsFile, vGtPoses);
    cv::Mat imgLeft;
    cv::Mat imgRight;
    for (std::size_t i = 0; i < 50/*vstrLeftImages.size()*/; i++) { // 4541 images
        readImages(imgLeft, imgRight, vstrLeftImages[i], vstrRightImages[i]);
        SLAM.trackStereo(imgLeft, imgRight, vTimestamps[i]);
    } //end for(int32_t i = 0; i < 4500; i++)

    SLAM.shutdown();

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
                const std::string &strRightImage) {
    imgLeft = cv::imread(strLeftImage, cv::IMREAD_GRAYSCALE);
    imgRight = cv::imread(strRightImage, cv::IMREAD_GRAYSCALE);

    if (imgLeft.empty()) {
        std::cerr << "Couldn't read image from" << strLeftImage << std::endl;
    } else if(imgRight.empty()) {
        std::cerr << "Couldn't read image from" << strRightImage << std::endl;
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
