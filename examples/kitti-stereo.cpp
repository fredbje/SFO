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
#include "oxts.h"

void loadImageFileNames(const std::string &strSequenceDir, std::vector<std::string> &vstrLeftImages,
                std::vector<std::string> &vstrRightImages);
void readImages(cv::Mat &imgLeft, cv::Mat &imgRight, const std::string &strLeftImage,
                const std::string &strRightImage);
void loadTimeStamps(const std::string &strTimestampsFile, std::vector<double> &vTimestamps);
void loadGtPoses(const std::string &strGtPosesFile, std::vector<libviso2::Matrix> &vGtPoses);
void loadOxtsData(const std::string &strOxtsDir, std::vector<oxts> &vOxtsData);

//////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv){
    if(argc != 6) {
        std::cerr << std::endl << "Usage: ./SFO_main path_to_sequence path_to_settings_file "
                "path_to_timestamps path_to_gt_poses path_to_navdata(oxts)" << std::endl;
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

    SFO::System SLAM(strSettingsFile, vOxtsData[0], vGtPoses);
    cv::Mat imgLeft;
    cv::Mat imgRight;
    for (std::size_t i = 0; i < 500/*vstrLeftImages.size()*/; i++) { // 4541 images
        readImages(imgLeft, imgRight, vstrLeftImages[i], vstrRightImages[i]);
        SLAM.trackStereo(imgLeft, imgRight, vTimestamps[i], vOxtsData[i]);
    } //end for(int32_t i = 0; i < 4500; i++)

    SLAM.shutdown();

    std::cout << "SFO_main complete! Exiting ..." << std::endl;

	return 0;
}

/////////////////////////////////////////////////////////////////////////////////////////////////

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

    // Data collected from KITTI calibration files
    libviso2::Matrix cam_T_velo(4, 4);
    cam_T_velo.val[0][0] = 7.967514e-03;  cam_T_velo.val[0][1] = -9.999679e-01; cam_T_velo.val[0][2] = -8.462264e-04; cam_T_velo.val[0][3] = -1.377769e-02;
    cam_T_velo.val[1][0] = -2.771053e-03; cam_T_velo.val[1][1] = 8.241710e-04;  cam_T_velo.val[1][2] = -9.999958e-01; cam_T_velo.val[1][3] = -5.542117e-02;
    cam_T_velo.val[2][0] = 9.999644e-01;  cam_T_velo.val[2][1] = 7.969825e-03;  cam_T_velo.val[2][2] = -2.764397e-03; cam_T_velo.val[2][3] = -2.918589e-01;
    cam_T_velo.val[3][0] = 0.0;           cam_T_velo.val[3][1] = 0.0;           cam_T_velo.val[3][2] = 0.0;           cam_T_velo.val[3][3] = 1.0;

    libviso2::Matrix velo_T_gps(4, 4);
    velo_T_gps.val[0][0] = 9.999976e-01;  velo_T_gps.val[0][1] = 7.553071e-04; velo_T_gps.val[0][2] = -2.035826e-03; velo_T_gps.val[0][3] = -8.086759e-01;
    velo_T_gps.val[1][0] = -7.854027e-04; velo_T_gps.val[1][1] = 9.998898e-01; velo_T_gps.val[1][2] = -1.482298e-02; velo_T_gps.val[1][3] = 3.195559e-01;
    velo_T_gps.val[2][0] = 2.024406e-03;  velo_T_gps.val[2][1] = 1.482454e-02; velo_T_gps.val[2][2] = 9.998881e-01;  velo_T_gps.val[2][3] = -7.997231e-01;
    velo_T_gps.val[3][0] = 0.0;           velo_T_gps.val[3][1] = 0.0;          velo_T_gps.val[3][2] = 0.0;           velo_T_gps.val[3][3] = 1.0;

    /*
    libviso2::Matrix transMat(4, 4);
    transMat.val[0][2] = 1;
    transMat.val[1][0] = -1;
    transMat.val[2][1] = -1;
    transMat.val[3][3] = 1;
    */

    libviso2::Matrix cam_T_gps = cam_T_velo * velo_T_gps;
    libviso2::Matrix gps_T_cam = libviso2::Matrix::inv(cam_T_gps);

    // Data collected from oxts data frame 1.
    libviso2::Matrix enu_R_gps = libviso2::Matrix::rotMatX(0.0)    // Roll
                                 * libviso2::Matrix::rotMatY(0.0)  // Pitch
                                 * libviso2::Matrix::rotMatZ(1.03959632679490); // Yaw

    libviso2::Matrix enu_T_gps = libviso2::Matrix(4, 4);
    enu_T_gps.setMat(enu_R_gps, 0, 0);
    enu_T_gps.val[3][3] = 1.0;

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
        tempPose = enu_T_gps * gps_T_cam * tempPose ;
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
    for(auto &direntryFile : bfs::directory_iterator(bfs::path(strOxtsDir))) {
        if (direntryFile.path().extension() == ".txt") {
            vstrFileNames.emplace_back(direntryFile.path().string());
        } else {
            std::cerr << direntryFile << " does not have a .txt extension." << std::endl;
            return;
        }
    }
    std::sort(vstrFileNames.begin(), vstrFileNames.end());
    for(const auto &strFileName : vstrFileNames) {

        std::ifstream f;
        std::string strLine;
        f.open(strFileName, std::ifstream::in);
        if(!f.is_open()){
            std::cerr << "Error, could not open " << strFileName << std::endl;
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
            std::cerr << "Could not read from " << strFileName << std::endl;
        }
        oxtsData.navsat = static_cast<int>(navsat);
        oxtsData.numsats = static_cast<int>(numsats);
        oxtsData.posmode = static_cast<int>(posmode);
        oxtsData.velmode = static_cast<int>(velmode);
        oxtsData.orimode = static_cast<int>(orimode);
        vOxtsData.push_back(oxtsData);
    }

}