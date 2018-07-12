#ifndef SFO_LOADFUNCTIONS_H
#define SFO_LOADFUNCTIONS_H

#include <vector>
#include <string>
#include <opencv2/core/mat.hpp>
#include "libviso2/matrix.h"
#include "oxts.h"

void loadImageFileNames(const std::string &strSequenceDir, std::vector<std::string> &vstrLeftImages,
                        std::vector<std::string> &vstrRightImages);
void loadImages(cv::Mat &imgLeft, cv::Mat &imgRight, const std::string &strLeftImage,
                const std::string &strRightImage);
void loadTimeStamps(const std::string &strTimestampsFile, std::vector<double> &vTimestamps);
void loadGtPoses(const std::string &strGtPosesFile, std::vector<libviso2::Matrix> &vGtPoses,
                 const libviso2::Matrix &imu_T_cam, const oxts &navdata0);
void loadOxtsData(const std::string &strOxtsDir, std::vector<oxts> &vOxtsData);
libviso2::Matrix loadCam2ImuTransform(const std::string &strImu2VeloCalibFile, const std::string &strVelo2CamCalibFile);

#endif //SFO_LOADFUNCTIONS_H
