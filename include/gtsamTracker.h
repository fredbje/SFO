//
// Created by lindeyang on 11/30/16.
//

#ifndef SFO_STEREO_H
#define SFO_STEREO_H
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/StereoPoint2.h>
#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/geometry/StereoCamera.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Marginals.h> // For noisemodel


#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <libviso2/matcher.h>
#include <libviso2/viso_stereo.h>
#include "libviso2/matrix.h"

namespace SFO {
    class GtsamTracker {
    public:

        GtsamTracker(const std::string &strSettingsFile);

        ~GtsamTracker();

        gtsam::Pose3 cvtMatrix2Pose3(const libviso2::Matrix &Min);
        libviso2::Matrix cvtPose32Matrix(const gtsam::Pose3 &Min);
        void cvtMatrix2RT(const libviso2::Matrix &Min, gtsam::Rot3 &R, gtsam::Point3 &T);
        void cvtgtPose2RT(const gtsam::Pose3 &pose, libviso2::Matrix &ptrM);

        void update(const libviso2::Matrix &T_delta, const std::vector<libviso2::Matcher::p_match> &vMatches,
                    const std::vector<int32_t> &vInliers);
        std::vector<libviso2::Matrix> optimize();

        // From the matched feature pair to previous and current sterepoints
        void getMatchedPairs(const libviso2::Matcher::p_match &match,
                          gtsam::StereoPoint2 &p1,
                          gtsam::StereoPoint2 &p2);
    private:
        size_t mPoseId, mLandmarkId;
        void loadCameraMatrix(const std::string &strSettingsFile);

        gtsam::Cal3_S2Stereo::shared_ptr mK;
        gtsam::Cal3_S2::shared_ptr mK1, mK2;
        gtsam::noiseModel::Isotropic::shared_ptr mMeasurementNoise2D, mMeasurementNoise3D;
        gtsam::noiseModel::Diagonal::shared_ptr mOdometryNoise;
        gtsam::StereoCamera mStereoCamera;

        gtsam::NonlinearFactorGraph mGraph;
        gtsam::Values mEstimate;
    };
}
#endif //SFO_STEREO_H
