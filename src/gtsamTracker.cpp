#define _USE_MATH_DEFINES
#include <cmath> // Can axess pi as M_PI
#include <fstream>
#include <iomanip> // setprecision

#include <gtsam/nonlinear/Marginals.h>
#include <opencv2/core/persistence.hpp>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <gtsam/slam/StereoFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/PoseTranslationPrior.h>
#include <gtsam/slam/PoseRotationPrior.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam_unstable/slam/SmartStereoProjectionPoseFactor.h>

#include "vertigo/betweenFactorSwitchable.h"
#include "vertigo/switchVariableLinear.h"

#include "gtsamTracker.h"

namespace SFO {
    GtsamTracker::GtsamTracker(const std::string &strSettingsFile, const oxts &navdata0, const libviso2::Matrix &imu_T_cam) {
        loadCameraMatrix(strSettingsFile);
        mProj.Reset(navdata0.lat, navdata0.lon, navdata0.alt);
        double x0, y0, z0;
        mProj.Forward(navdata0.lat, navdata0.lon, navdata0.alt, x0, y0, z0);
        std::cout << "Initial global coordinates: " << std::setprecision(14) << navdata0.lat << ", " << navdata0.lon << ", " << navdata0.alt << std::endl;
        std::cout << "Initial local coordinates: " << x0 << ", " << y0 << ", " << z0 << std::endl;

        mPoseId = 0;

        gtsam::Rot3 enu_R_imu = gtsam::Rot3::RzRyRx(navdata0.roll, navdata0.pitch, navdata0.yaw);
        gtsam::Point3 enu_t_imu = gtsam::Point3(0, 0, 0);
        gtsam::Pose3 enu_T_imu = gtsam::Pose3(enu_R_imu, enu_t_imu);
        gtsam::Pose3 initialPoseEstimate = enu_T_imu * cvtMatrix2Pose3(imu_T_cam);
        mNewValues.insert(gtsam::Symbol('x', mPoseId), initialPoseEstimate);

        //gtsam::noiseModel::Diagonal::shared_ptr priorTranslationNoise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(navdata0.pos_accuracy, navdata0.pos_accuracy, navdata0.pos_accuracy)); // 1 m in x, y and z
        //mGraph.emplace_shared<gtsam::PoseTranslationPrior<gtsam::Pose3> >(gtsam::Symbol('x', mPoseId), initialPoseEstimate, priorTranslationNoise); // No prior on rotation. Translation prior uses only translation part of pose

        gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << gtsam::Vector3(0.2, 0.2, 0.2), gtsam::Vector3(navdata0.pos_accuracy, navdata0.pos_accuracy, navdata0.pos_accuracy)).finished()); // Assuming 0.2 rad in roll, pitch, yaw
        mNewFactors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(gtsam::Symbol('x', mPoseId), initialPoseEstimate, priorPoseNoise);

        mPoseId++;

        mOdometryNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << gtsam::Vector3::Constant(0.1), gtsam::Vector3::Constant(0.05)).finished()); // 10cm std on x,y,z 0.05 rad on roll,pitch,yaw

        //mGraph.emplace_shared<gtsam::NonlinearEquality<gtsam::Pose3> >(gtsam::Symbol('x',1), mStereoCamera.pose());


        mParameters.relinearizeThreshold = 0.01;
        mParameters.relinearizeSkip = 1;
        mIsam = gtsam::ISAM2(mParameters);
        mIsam.update(mNewFactors, mNewValues);
        mNewFactors.resize(0);
        mNewValues.clear();

    }

    GtsamTracker::~GtsamTracker() {
        std::cout << "GtsamTracker destructor called." << std::endl;
    }


    void GtsamTracker::getMatchedPairs(const libviso2::Matcher::p_match &match,
                      gtsam::StereoPoint2 &spt1,
                      gtsam::StereoPoint2 &spt2){
        spt1 = gtsam::StereoPoint2(match.u1p, match.u2p, match.v1p);
        spt2 = gtsam::StereoPoint2(match.u1c, match.u2c, match.v1c);
    }

    void GtsamTracker::update(const libviso2::Matrix &T_delta, const std::vector<libviso2::Matcher::p_match> &vMatches,
                              const std::vector<int32_t> &vInliers, const oxts &navdata) {

/*
        const gtsam::noiseModel::Isotropic::shared_ptr imageNoiseModel = gtsam::noiseModel::Isotropic::Sigma(3, 1); // 1 pixel in ul, ur, v
        gtsam::Pose3 pose3T_delta = cvtMatrix2Pose3(T_delta);

        gtsam::SmartStereoProjectionPoseFactor::shared_ptr factor(new gtsam::SmartStereoProjectionPoseFactor(imageNoiseModel));
        gtsam::StereoPoint2 spt1, spt2;
        for(size_t kk = 0; kk < vInliers.size(); kk++) {
            getMatchedPairs(vMatches[vInliers[kk]], spt1, spt2);
            mGraph.push_back(factor);
            factor = gtsam::SmartStereoProjectionPoseFactor::shared_ptr(new gtsam::SmartStereoProjectionPoseFactor(imageNoiseModel));
            factor->add(spt1, gtsam::Symbol('x', mPoseId-1), mK);
            factor->add(spt2, gtsam::Symbol('x', mPoseId), mK);
        }

*/


        if((mPoseId % 100) == 0) {
            double x, y, z;
            mProj.Forward(navdata.lat, navdata.lon, navdata.alt, x, y, z);
            gtsam::noiseModel::Diagonal::shared_ptr priorTranslationNoise = gtsam::noiseModel::Diagonal::Sigmas(
                    gtsam::Vector3(navdata.pos_accuracy, navdata.pos_accuracy, navdata.pos_accuracy)); // pos_accuracy[m] on x, y, z
            mNewFactors.emplace_shared<gtsam::PoseTranslationPrior<gtsam::Pose3> >(gtsam::Symbol('x', mPoseId),
                                                                              gtsam::Point3(x, y, z),
                                                                              priorTranslationNoise);
        }

        gtsam::Pose3 pose3T_delta = cvtMatrix2Pose3(T_delta);
        mNewFactors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3> >(gtsam::Symbol('x', mPoseId-1), gtsam::Symbol('x', mPoseId), pose3T_delta, mOdometryNoise);
        //mStereoCamera = gtsam::StereoCamera(gtsamPose, mK);
        gtsam::Pose3 lastPose;

        lastPose = mIsam.calculateEstimate<gtsam::Pose3>(gtsam::Symbol('x', mPoseId-1));

        mNewValues.insert(gtsam::Symbol('x', mPoseId++), lastPose * pose3T_delta);

        /*
        gtsam::StereoPoint2 spt1, spt2;
        for(size_t kk=0; kk<vInliers.size()-1; kk++) {
            getMatchedPairs(vMatches[vInliers[kk]], spt1, spt2);

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


        mIsam.update(mNewFactors, mNewValues);
        //mIsam.update();
        mCurrentEstimate = mIsam.calculateEstimate();
        mNewFactors.resize(0);
        mNewValues.clear();

/*
        gtsam::LevenbergMarquardtParams params;
        //params.orderingType = gtsam::Ordering::METIS;
        params.maxIterations = 1000;
        mEstimate = gtsam::LevenbergMarquardtOptimizer(mGraph, mEstimate, params).optimize();
        //mEstimate = gtsam::LevenbergMarquardtOptimizer(mGraph, mEstimate).optimize();

*/


        std::vector<libviso2::Matrix> poses;
        for(size_t i = 0; i < mPoseId; i++) {
            gtsam::Pose3 tempPose = mCurrentEstimate.at<gtsam::Pose3>(gtsam::Symbol('x', i));
            poses.push_back(cvtPose32Matrix(tempPose));
        }
        return poses;

    }


    gtsam::Pose3 GtsamTracker::cvtMatrix2Pose3(const libviso2::Matrix &Min) {
        auto R = gtsam::Rot3(Min.val[0][0], Min.val[0][1],Min.val[0][2],
                        Min.val[1][0], Min.val[1][1],Min.val[1][2],
                        Min.val[2][0], Min.val[2][1],Min.val[2][2]);
        auto t = gtsam::Point3(Min.val[0][3], Min.val[1][3], Min.val[2][3]);

        gtsam::Pose3 Mout(R, t);
        return Mout;
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

    void GtsamTracker::save() {
        std::cout << "Saving GPS track to file..." << std::endl;
        std::ofstream f;
        f.open("output.txt");

        for(size_t i = 0; i < mPoseId; i++) {
            gtsam::Pose3 tempPose = mCurrentEstimate.at<gtsam::Pose3>(gtsam::Symbol('x', i));
            double lat, lon, h;
            mProj.Reverse(tempPose.x(), tempPose.y(), tempPose.z(), lat, lon, h);
            f << std::setprecision(14) << lat << " " << lon << "\n";
        }
    }
}
