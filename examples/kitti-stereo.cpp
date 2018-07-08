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

struct oxts { // oxts is the brand of the INS used in the KITTI dataset
    /*
    lat:           latitude of the oxts-unit (deg)
    lon:           longitude of the oxts-unit (deg)
    alt:           altitude of the oxts-unit (m)
    roll:          roll angle (rad),    0 = level, positive = left side up,      range: -pi   .. +pi
    pitch:         pitch angle (rad),   0 = level, positive = front down,        range: -pi/2 .. +pi/2
    yaw:           heading (rad),       0 = east,  positive = counter clockwise, range: -pi   .. +pi
    vn:            velocity towards north (m/s)
    ve:            velocity towards east (m/s)
    vf:            forward velocity, i.e. parallel to earth-surface (m/s)
    vl:            leftward velocity, i.e. parallel to earth-surface (m/s)
    vu:            upward velocity, i.e. perpendicular to earth-surface (m/s)
    ax:            acceleration in x, i.e. in direction of vehicle front (m/s^2)
    ay:            acceleration in y, i.e. in direction of vehicle left (m/s^2)
    ay:            acceleration in z, i.e. in direction of vehicle top (m/s^2)
    af:            forward acceleration (m/s^2)
    al:            leftward acceleration (m/s^2)
    au:            upward acceleration (m/s^2)
    wx:            angular rate around x (rad/s)
    wy:            angular rate around y (rad/s)
    wz:            angular rate around z (rad/s)
    wf:            angular rate around forward axis (rad/s)
    wl:            angular rate around leftward axis (rad/s)
    wu:            angular rate around upward axis (rad/s)
    pos_accuracy:  velocity accuracy (north/east in m)
    vel_accuracy:  velocity accuracy (north/east in m/s)
    navstat:       navigation status (see navstat_to_string)
    numsats:       number of satellites tracked by primary GPS receiver
    posmode:       position mode of primary GPS receiver (see gps_mode_to_string)
    velmode:       velocity mode of primary GPS receiver (see gps_mode_to_string)
    orimode:       orientation mode of primary GPS receiver (see gps_mode_to_string)
    */
    double lat, lon, alt,
            roll, pitch, yaw,
            vn, ve, vf, vl, vu,
            ax, ay, az, af, al, au,
            wx, wy, wz, wf, wl, wu,
            pos_accuracy, vel_accuracy;
    int navsat, numsats,
        posmode, velmode, orimode;
};

void loadImageFileNames(const std::string &strSequenceDir, std::vector<std::string> &vstrLeftImages,
                std::vector<std::string> &vstrRightImages);
void readImages(cv::Mat &imgLeft, cv::Mat &imgRight, const std::string &strLeftImage,
                const std::string &strRightImage);
void loadTimeStamps(const std::string &strTimestampsFile, std::vector<double> &vTimestamps);
void loadGtPoses(const std::string &strGtPosesFile, std::vector<libviso2::Matrix> &vGtPoses);
void loadOxtsData(const std::string &strOxtsDir, std::vector<oxts> &vOxtsData);

int main(int argc, char **argv){
    if(argc != 6) {
        std::cerr << std::endl << "Usage: ./SFO_main path_to_sequence path_to_settings_file "
                "path_to_timestamps path_to_gt_poses path_to_oxts" << std::endl;
        return 1;
    }

    std::string strSequenceDir = argv[1];
    std::string strSettingsFile = argv[2];
    std::string strTimestampsFile = argv[3];
    std::string strGtPosesFile = argv[4];
    std::string strOxtsDir = argv[5];

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

    std::vector<oxts> vOxtsData;
    loadOxtsData(strOxtsDir, vOxtsData);

/////////////////////////////////////////////


    SFO::System SLAM(strSettingsFile, vGtPoses);
    cv::Mat imgLeft;
    cv::Mat imgRight;
    for (std::size_t i = 0; i < 500/*vstrLeftImages.size()*/; i++) { // 4541 images
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



void loadOxtsData(const std::string &strOxtsDir, std::vector<oxts> &vOxtsData){
    if(!bfs::is_directory(strOxtsDir)) {
        std::cerr << strOxtsDir << " is not a directory." << std::endl;
        return;
    }
    std::vector<std::string> vstrFileNames;
    for(auto& direntryFile : bfs::directory_iterator(bfs::path(strOxtsDir))) {
        if (direntryFile.path().extension() == ".txt") {
            vstrFileNames.emplace_back(direntryFile.path().string());
        } else {
            std::cerr << direntryFile << " does not have a .txt extension." << std::endl;
            return;
        }
    }
    std::sort(vstrFileNames.begin(), vstrFileNames.end());
    for(auto it = vstrFileNames.begin(); it != vstrFileNames.end(); ++it) {

        std::ifstream f;
        std::string strLine;
        f.open(*it, std::ifstream::in);
        if(!f.is_open()){
            std::cerr << "Error, could not open " << *it << std::endl;
            return;
        }
        std::getline(f, strLine);
        std::istringstream iss(strLine);
        oxts oxtsData;
        double navsat, numsats, posmode, velmode, orimode;
        if (!(iss >> oxtsData.lat
                  >> oxtsData.lon
                  >> oxtsData.alt
                  >> oxtsData.roll
                  >> oxtsData.pitch
                  >> oxtsData.yaw
                  >> oxtsData.vn
                  >> oxtsData.ve
                  >> oxtsData.vf
                  >> oxtsData.vl
                  >> oxtsData.vu
                  >> oxtsData.ax
                  >> oxtsData.ay
                  >> oxtsData.az
                  >> oxtsData.af
                  >> oxtsData.al
                  >> oxtsData.au
                  >> oxtsData.wx
                  >> oxtsData.wy
                  >> oxtsData.wz
                  >> oxtsData.wf
                  >> oxtsData.wl
                  >> oxtsData.wu
                  >> oxtsData.pos_accuracy
                  >> oxtsData.vel_accuracy
                  >> navsat
                  >> numsats
                  >> posmode
                  >> velmode
                  >> orimode)) {
            std::cerr << "Could not read from " << *it << std::endl;
        }
        oxtsData.navsat = static_cast<int>(navsat);
        oxtsData.numsats = static_cast<int>(numsats);
        oxtsData.posmode = static_cast<int>(posmode);
        oxtsData.velmode = static_cast<int>(velmode);
        oxtsData.orimode = static_cast<int>(orimode);
        vOxtsData.push_back(oxtsData);
    }

}