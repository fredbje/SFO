#include<iostream>

#include "libviso2/matrix.h"
#include "libviso2/viso_stereo.h"

#include "gtsamTracker.h"
#include "mapDrawer.h"
#include "frameDrawer.h"
#include "system.h"
#include "loadFunctions.h"
#include "oxts.h"
//#include "tracking.h"

int main(int argc, char **argv){
    std::string strSequenceDir = "/home/fredbje/Datasets/kitti-gray/sequences/00";
    std::string strSettingsFile = "/home/fredbje/git/SFO/examples/KITTI00-02.yaml";
    std::string strTimestampsFile = "/home/fredbje/Datasets/kitti-gray/sequences/00/times.txt";
    std::string strGtPosesFile = "/home/fredbje/Datasets/kitti-poses/dataset/poses/00.txt";
    std::string strOxtsDir = "/home/fredbje/Datasets/2011_10_03/2011_10_03_drive_0027_sync/oxts/data";
    std::string strImu2VeloCalibFile = "/home/fredbje/Datasets/2011_10_03/2011_10_03/calib_imu_to_velo.txt";
    std::string strVelo2CamCalibFile = "/home/fredbje/Datasets/2011_10_03/2011_10_03/calib_velo_to_cam.txt";
    std::string strVocabularyFile = "/home/fredbje/git/SFO/vocabulary/ORBvoc.txt";

    std::vector<std::string> vstrLeftImages;
    std::vector<std::string> vstrRightImages;
    loadImageFileNames(strSequenceDir, vstrLeftImages, vstrRightImages);

    std::vector<double> vTimestamps;
    loadTimeStamps(strTimestampsFile, vTimestamps);

    if(vTimestamps.size() != vstrLeftImages.size()) {
        std::cerr << "Number of timestamps does not match number of images" << std::endl;
        return 1;
    }

    std::vector<oxts> vOxtsData;
    loadOxtsData(strOxtsDir, vOxtsData);

    libviso2::Matrix imu_T_cam = loadCam2ImuTransform(strImu2VeloCalibFile, strVelo2CamCalibFile);

    std::vector<libviso2::Matrix> vGtPoses;
    loadGtPoses(strGtPosesFile, vGtPoses, imu_T_cam, vOxtsData[0]);

    if(vGtPoses.size() != vstrLeftImages.size()) {
        std::cerr << "Number of GT poses does not match number of images." << std::endl;
        return 1;
    }

    SFO::System SLAM(strSettingsFile, strVocabularyFile, vOxtsData[0], imu_T_cam, vGtPoses);
    cv::Mat imgLeft;
    cv::Mat imgRight;
    //SFO::Tracking tracker(strSettingsFile);
    for (std::size_t i = 0; i < vstrLeftImages.size(); i++) { // 4541 images
        loadImages(imgLeft, imgRight, vstrLeftImages[i], vstrRightImages[i]);
        //tracker.track(imgLeft, imgRight, 0);
        SLAM.trackStereo(imgLeft, imgRight, vTimestamps[i], vOxtsData[i]);
    }

    SLAM.shutdown();

    std::cout << "SFO_main complete! Exiting ..." << std::endl;

	return 0;
}