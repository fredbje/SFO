#ifndef SFO_GTSAMTRACKER_H
#define SFO_GTSAMTRACKER_H

#include <iostream>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/StereoPoint2.h>
#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/geometry/StereoCamera.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Marginals.h> // For noisemodel
#include <gtsam/nonlinear/ISAM2.h>

#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/Geocentric.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <libviso2/matcher.h>
#include <libviso2/viso_stereo.h>
#include "libviso2/matrix.h"

#include "oxts.h"
#include "loopDetector.h"

namespace SFO {
    class GtsamTracker {
    public:

        GtsamTracker(const std::string &strSettingsFile, const oxts &navdata0, const libviso2::Matrix &imu_T_cam);

        ~GtsamTracker();

        gtsam::Pose3 cvtMatrix2Pose3(const libviso2::Matrix &Min);
        gtsam::Rot3 cvtMatrix2Rot3(const cv::Mat &Rin);
        gtsam::Point3 cvtMatrix2Point3(const cv::Mat &tin);
        libviso2::Matrix cvtPose32Matrix(const gtsam::Pose3 &Min);
        void cvtMatrix2RT(const libviso2::Matrix &Min, gtsam::Rot3 &R, gtsam::Point3 &T);
        void cvtgtPose2RT(const gtsam::Pose3 &pose, libviso2::Matrix &ptrM);

        void update(const libviso2::Matrix &T_delta,
                    const std::vector<libviso2::Matcher::p_match> &vMatches,
                    const std::vector<int32_t> &vInliers,
                    const oxts &navdata,
                    const DLoopDetector::DetectionResult &loopResult);

        std::vector<libviso2::Matrix> optimize();

        // From the matched feature pair to previous and current sterepoints
        void getMatchedPairs(const libviso2::Matcher::p_match &match,
                          gtsam::StereoPoint2 &p1,
                          gtsam::StereoPoint2 &p2);

        void save();
    private:
        size_t mPoseId, mLandmarkId, mSwitchId;
        void loadCameraMatrix(const std::string &strSettingsFile);

        gtsam::Cal3_S2Stereo::shared_ptr mK;
        gtsam::Cal3_S2::shared_ptr mK1, mK2;
        gtsam::noiseModel::Isotropic::shared_ptr mMeasurementNoise2D, mMeasurementNoise3D;
        gtsam::noiseModel::Diagonal::shared_ptr mOdometryNoise;
        gtsam::StereoCamera mStereoCamera;

        gtsam::NonlinearFactorGraph mNewFactors;
        gtsam::Values mNewValues;
        gtsam::Values mCurrentEstimate;

        gtsam::ISAM2Params mParameters;
        gtsam::ISAM2 mIsam;

        DLoopDetector::DetectionResult mLoopResult;

        //GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f());
        GeographicLib::LocalCartesian mProj;
    };
}
#endif //SFO_GTSAMTRACKER_H
