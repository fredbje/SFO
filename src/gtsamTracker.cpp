#include <gtsam/nonlinear/Marginals.h>
#include <opencv2/core/persistence.hpp>


#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>



#include <gtsam/slam/StereoFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>


#include "gtsamTracker.h"

namespace SFO {
    GtsamTracker::GtsamTracker(const std::string &strSettingsFile) {
        loadCameraMatrix(strSettingsFile);
        mMeasurementNoise3D = gtsam::noiseModel::Isotropic::Sigma(3, 1);
        mMeasurementNoise2D = gtsam::noiseModel::Isotropic::Sigma(2, 1); // 1 pixel in u and v
        gtsam::Pose3 initialPose; // Origin is default constructor
        mStereoCamera = gtsam::StereoCamera(initialPose, mK);
        mPoseId = 0;
        mLandmarkId = 1;
        //mEstimate.insert(gtsam::Symbol('x', mPoseId++), mStereoCamera.pose());

        gtsam::Pose3 priorMean; // Origin is default constructor
        gtsam::noiseModel::Diagonal::shared_ptr priorNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << gtsam::Vector3::Constant(0.3), gtsam::Vector3::Constant(0.1)).finished()); // 30cm std on x,y,z 0.1 rad on roll,pitch,yaw
        mOdometryNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << gtsam::Vector3::Constant(0.1), gtsam::Vector3::Constant(0.05)).finished()); // 10cm std on x,y,z 0.05 rad on roll,pitch,yaw
        mEstimate.insert(gtsam::Symbol('x', mPoseId), priorMean);
        mGraph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3> >(gtsam::Symbol('x', mPoseId++), priorMean, priorNoise);


        //mGraph.emplace_shared<gtsam::NonlinearEquality<gtsam::Pose3> >(gtsam::Symbol('x',1), mStereoCamera.pose());
    }

    GtsamTracker::~GtsamTracker() = default;


    void GtsamTracker::getMatchedPairs(const libviso2::Matcher::p_match &match,
                      gtsam::StereoPoint2 &spt1,
                      gtsam::StereoPoint2 &spt2){
        spt1 = gtsam::StereoPoint2(match.u1p, match.u2p, match.v1p);
        spt2 = gtsam::StereoPoint2(match.u1c, match.u2c, match.v1c);
    }

    void GtsamTracker::update(const libviso2::Matrix &T_delta, const std::vector<libviso2::Matcher::p_match> &vMatches,
                              const std::vector<int32_t> &vInliers) {
        // TODO adjust noise based on outlier to inlier ratio (or something)
        gtsam::Pose3 pose3T_delta = cvtMatrix2Pose3(T_delta);
        mGraph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3> >(gtsam::Symbol('x', mPoseId), gtsam::Symbol('x', mPoseId-1), pose3T_delta, mOdometryNoise); // TODO Why do we need to reverse the direction?
        //mStereoCamera = gtsam::StereoCamera(gtsamPose, mK);
        gtsam::Pose3 lastPose = mEstimate.at<gtsam::Pose3>(gtsam::Symbol('x', mPoseId-1));
        mEstimate.insert(gtsam::Symbol('x', mPoseId++), pose3T_delta * lastPose);

        /*
        gtsam::StereoPoint2 spt1, spt2;
        for(size_t kk=0; kk<vInliers.size()-1; kk++) { // TODO remove -1?
            getMatchedPairs(vMatches[vInliers[kk]], spt1, spt2);
            // TODO find id of landmark. Landmarks need a global id.


            mGraph.emplace_shared<
            gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>>(spt1, mMeasurementNoise3D,
                                                                             gtsam::Symbol('x', mPoseId - 1),
                                                                             gtsam::Symbol('l', mLandmarkId), mK);
            mGraph.emplace_shared<
            gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>>(spt2, mMeasurementNoise3D,
                                                                             gtsam::Symbol('x', mPoseId),
                                                                             gtsam::Symbol('l', mLandmarkId++), mK);


            mGraph.emplace_shared<
                    gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3>>(spt1, mMeasurementNoise3D,
                                                                             gtsam::Symbol('x', mPoseId - 1),
                                                                             gtsam::Symbol('l', mLandmarkId), mK);
            mGraph.emplace_shared<
                    gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3>>(spt2, mMeasurementNoise3D,
                                                                             gtsam::Symbol('x', mPoseId),
                                                                             gtsam::Symbol('l', mLandmarkId), mK);


            if (!mEstimate.exists(gtsam::Symbol('l', mLandmarkId))) {
                gtsam::Point3 worldPoint = mStereoCamera.backproject(spt2);
                mEstimate.insert(gtsam::Symbol('l', mLandmarkId), worldPoint);
            }



            gtsam::Point3 pt = stereoCam.backproject(spt1);
            if (pt.z() <= 40 * (mK->baseline())) {
                mGraph.emplace_shared<
                    gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3>>(spt1, mMeasurementNoise,
                                                                             gtsam::Symbol('x', mPoseId - 1),
                                                                             gtsam::Symbol('l', mLandmarkId), mK);
            } else {
                mGraph.emplace_shared<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>>(measurement, measurementNoise, Symbol('x', i), Symbol('l', j), K);
            }

            mLandmarkId++;
        }
        mPoseId++;
*/
    }

    std::vector<libviso2::Matrix> GtsamTracker::optimize() {
        mEstimate = gtsam::LevenbergMarquardtOptimizer(mGraph, mEstimate).optimize();
        std::vector<libviso2::Matrix> poses;
        for(size_t i = 0; i < mPoseId; i++) {
            gtsam::Pose3 tempPose = mEstimate.at<gtsam::Pose3>(gtsam::Symbol('x', i));
            poses.push_back(cvtPose32Matrix(tempPose));
        }
        return poses;

        /*
        gtsam::ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.01;
        parameters.relinearizeSkip = 1;
        gtsam::ISAM2 isam(parameters);

        isam.update(mGraph, mEstimate);
        // Each call to iSAM2 update(*) performs one iteration of the iterative nonlinear solver.
        // If accuracy is desired at the expense of time, update(*) can be called additional times
        // to perform multiple optimizer iterations every step.
        //isam.update();
        gtsam::Values mEstimate = isam.calculateEstimate();


        //gtsam::LevenbergMarquardtParams params;
        //params.orderingType = gtsam::Ordering::METIS;
        //params.maxIterations = 10;
        //gtsam::LevenbergMarquardtOptimizer optimizer(mGraph, mEstimate, params);
        //mEstimate = optimizer.optimize();

        std::vector<libviso2::Matrix> poses;
        for(size_t i = 0; i < mPoseId; i++) {
            gtsam::Pose3 tempPose = mEstimate.at<gtsam::Pose3>(gtsam::Symbol('x',i));
            poses.push_back(cvtGtsam2Matrix(tempPose.matrix()));
        }
        return poses;
        */
    }


    gtsam::Pose3 GtsamTracker::cvtMatrix2Pose3(const libviso2::Matrix &Min) {
        long m = Min.m;
        long n = Min.n;

        gtsam::Matrix Mout(Min.m, Min.n);

        for (int i=0;i<m; i++){
            for (int j=0;j<n;j++){
                Mout(i,j) = Min.val[i][j];
            }
        }
        return gtsam::Pose3(Mout);
    }


    libviso2::Matrix GtsamTracker::cvtPose32Matrix(const gtsam::Pose3 &Min) {
        libviso2::Matrix Mout(4, 4);
        for (int i=0;i<Min.matrix().rows(); i++){
            for (int j=0;j<Min.matrix().cols();j++){
                Mout.val[i][j] = Min.matrix()(i, j);
            }
        }
        return Mout;
    }


    void GtsamTracker::loadCameraMatrix(const std::string &strSettingsFile) {
        cv::FileStorage fSettings(strSettingsFile, cv::FileStorage::READ);

        if(!fSettings.isOpened()) {
            std::cerr << "Failed to open settings file at: " << strSettingsFile << std::endl;
        }

        float fx = fSettings["Camera.fx"]; // focal length in pixels
        float fy = fSettings["Camera.fy"]; // focal length in pixels
        float cx = fSettings["Camera.cx"]; // principal point (x/u-coordinate) in pixels
        float cy = fSettings["Camera.cy"]; // principal point (y/v-coordinate) in pixels
        float bf = fSettings["Camera.bf"];
        float s  = fSettings["Camera.s"];  // pixel shear
        float base = bf/fx;                // baseline in meters


        // construct the stereo calibration shared pointer, no need to delete it
        mK = gtsam::Cal3_S2Stereo::shared_ptr(new gtsam::Cal3_S2Stereo(fx, fy, s, cx, cy, base));
        mK1 = gtsam::Cal3_S2::shared_ptr(new gtsam::Cal3_S2(fx, fy, s, cx, cy));
        mK2 = gtsam::Cal3_S2::shared_ptr(new gtsam::Cal3_S2(fx, fy, s, cx, cy));
    }
}
