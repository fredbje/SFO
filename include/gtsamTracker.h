//
// Created by lindeyang on 11/30/16.
//

#ifndef SFO_STEREO_H
#define SFO_STEREO_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <libviso2/matcher.h>
#include <libviso2/viso_stereo.h>
#include "libviso2/matrix.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>

#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/geometry/StereoCamera.h>
#include <gtsam/geometry/StereoPoint2.h>
#include <gtsam/slam/StereoFactor.h>

namespace SFO {
    class GtsamTracker {
    public:

        GtsamTracker(const std::string &strSettingsFile);

        ~GtsamTracker();

        void cvtMatrix2Gtsam(const libviso2::Matrix &Min, gtsam::Matrix &Mout);
        void cvtMatrix2RT(const libviso2::Matrix &Min, gtsam::Rot3 &R, gtsam::Point3 &T);
        void cvtgtPose2RT(const gtsam::Pose3 &pose, libviso2::Matrix &ptrM);

        void update(libviso2::Matrix, const std::vector<libviso2::Matcher::p_match> &vMatches,
                    const std::vector<int32_t> &vInliers);
        void optimize(std::vector<libviso2::Matrix> *gtsamPoses);

        // From the matched feature pair to previous and current sterepoints
        void getMatchedPairs(const libviso2::Matcher::p_match &match,
                          gtsam::StereoPoint2 &p1,
                          gtsam::StereoPoint2 &p2);
    private:
        size_t mPoseId, mLandmarkId;
        void loadCameraMatrix(const std::string &strSettingsFile);
        gtsam::Cal3_S2Stereo::shared_ptr mK;
        //gtsam::noiseModel::Isotropic::shared_ptr mNoiseModel;
        gtsam::noiseModel::Isotropic::shared_ptr mMeasurementNoise;
        gtsam::StereoCamera mStereoCamera;

        gtsam::NonlinearFactorGraph mGraph;
        gtsam::Values mInitialEstimate;
    };
}
#endif //SFO_STEREO_H
