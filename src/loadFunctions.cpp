#include <iostream>
#include <fstream>
#include <sstream>
#include <boost/filesystem.hpp>
#include <opencv2/highgui.hpp>
#include "loadFunctions.h"

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

void loadImages(cv::Mat &imgLeft, cv::Mat &imgRight, const std::string &strLeftImage,
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

void loadGtPoses(const std::string &strGtPosesFile, std::vector<libviso2::Matrix> &vGtPoses,
                 const libviso2::Matrix &imu_T_cam, const oxts &navdata0) {
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

    // Data collected from oxts data frame 1.
    libviso2::Matrix enu_R_imu = libviso2::Matrix::rotMatX(navdata0.roll)
                                 * libviso2::Matrix::rotMatY(navdata0.pitch)
                                 * libviso2::Matrix::rotMatZ(navdata0.yaw);

    libviso2::Matrix enu_T_imu = libviso2::Matrix(4, 4);
    enu_T_imu.setMat(enu_R_imu, 0, 0);
    enu_T_imu.val[3][3] = 1.0;

    while(std::getline(f, strLine)) {
        std::istringstream iss(strLine);
        double r11, r12, r13, r21, r22, r23, r31, r32, r33, t1, t2, t3;

        if(!(iss >> r11 >> r12 >> r13 >> t1 >> r21 >> r22 >> r23 >> t2 >> r31 >> r32 >> r33 >> t3)) {
            break;
        }

        libviso2::Matrix tempPose(4,4);
        tempPose.val[0][0] = r11; tempPose.val[0][1] = r12; tempPose.val[0][2] = r13; tempPose.val[0][3] = t1;
        tempPose.val[1][0] = r21; tempPose.val[1][1] = r22; tempPose.val[1][2] = r23; tempPose.val[1][3] = t2;
        tempPose.val[2][0] = r31; tempPose.val[2][1] = r32; tempPose.val[2][2] = r33; tempPose.val[2][3] = t3;
        tempPose.val[3][0] = 0.0; tempPose.val[3][1] = 0.0; tempPose.val[3][2] = 0.0; tempPose.val[3][3] = 1.0;
        //tempPose = enu_T_imu * imu_T_cam * tempPose ;
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

libviso2::Matrix loadCam2ImuTransform(const std::string &strImu2VeloCalibFile, const std::string &strVelo2CamCalibFile){
    if(!bfs::is_regular_file(strImu2VeloCalibFile)) {
        std::cerr << strImu2VeloCalibFile << " is not a file." << std::endl;
        return libviso2::Matrix();
    } else if(!bfs::is_regular_file(strVelo2CamCalibFile)) {
        std::cerr << strVelo2CamCalibFile << " is not a file." << std::endl;
        return libviso2::Matrix();
    }

    libviso2::Matrix velo_T_imu(4, 4);
    {
        std::ifstream f;
        std::string strLine;
        f.open(strImu2VeloCalibFile.c_str(), std::ifstream::in);
        if (!f.is_open()) {
            std::cerr << "Could not open " << strImu2VeloCalibFile << std::endl;
        }

        if (!std::getline(f, strLine)) {
            std::cerr << "Could not read from " << strImu2VeloCalibFile << std::endl;
            return libviso2::Matrix();
        }

        std::istringstream iss(strLine);
        double r11, r12, r13, r21, r22, r23, r31, r32, r33, t1, t2, t3;

        if (!(iss >> r11 >> r12 >> r13 >> t1 >> r21 >> r22 >> r23 >> t2 >> r31 >> r32 >> r33 >> t3)) {
            std::cerr << "Could not read stream from " << strImu2VeloCalibFile << std::endl;
        }

        velo_T_imu.val[0][0] = r11; velo_T_imu.val[0][1] = r12; velo_T_imu.val[0][2] = r13; velo_T_imu.val[0][3] = t1;
        velo_T_imu.val[1][0] = r21; velo_T_imu.val[1][1] = r22; velo_T_imu.val[1][2] = r23; velo_T_imu.val[1][3] = t2;
        velo_T_imu.val[2][0] = r31; velo_T_imu.val[2][1] = r32; velo_T_imu.val[2][2] = r33; velo_T_imu.val[2][3] = t3;
        velo_T_imu.val[3][0] = 0.0; velo_T_imu.val[3][1] = 0.0; velo_T_imu.val[3][2] = 0.0; velo_T_imu.val[3][3] = 1.0;
    }

    libviso2::Matrix cam_T_velo(4, 4);
    {
        std::ifstream f;
        std::string strLine;
        f.open(strVelo2CamCalibFile.c_str(), std::ifstream::in);
        if (!std::getline(f, strLine)) {
            std::cerr << "Could not read from " << strVelo2CamCalibFile << std::endl;
            return libviso2::Matrix();
        }

        std::istringstream iss(strLine);
        double r11, r12, r13, r21, r22, r23, r31, r32, r33, t1, t2, t3;

        if (!(iss >> r11 >> r12 >> r13 >> t1 >> r21 >> r22 >> r23 >> t2 >> r31 >> r32 >> r33 >> t3)) {
            std::cerr << "Could not read stream from " << strImu2VeloCalibFile << std::endl;
        }

        cam_T_velo.val[0][0] = r11; cam_T_velo.val[0][1] = r12; cam_T_velo.val[0][2] = r13; cam_T_velo.val[0][3] = t1;
        cam_T_velo.val[1][0] = r21; cam_T_velo.val[1][1] = r22; cam_T_velo.val[1][2] = r23; cam_T_velo.val[1][3] = t2;
        cam_T_velo.val[2][0] = r31; cam_T_velo.val[2][1] = r32; cam_T_velo.val[2][2] = r33; cam_T_velo.val[2][3] = t3;
        cam_T_velo.val[3][0] = 0.0; cam_T_velo.val[3][1] = 0.0; cam_T_velo.val[3][2] = 0.0; cam_T_velo.val[3][3] = 1.0;
    }


    libviso2::Matrix cam_T_imu = cam_T_velo * velo_T_imu;
    libviso2::Matrix imu_T_cam = libviso2::Matrix::inv(cam_T_imu);
    std::cout << "Loaded camera to imu frame transformation." << std::endl;
    return imu_T_cam;

}