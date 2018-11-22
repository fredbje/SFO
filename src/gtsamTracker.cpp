#define _USE_MATH_DEFINES
#include <cmath> // Can axess pi as M_PI
#include <fstream>
#include <iomanip> // setprecision

#include <gtsam/nonlinear/Marginals.h>
#include <opencv2/core/persistence.hpp>
#include <opencv2/calib3d/calib3d.hpp> // For findEssentialMat

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
#include <gtsam/slam/BearingFactor.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>
#include <gtsam/geometry/EssentialMatrix.h>
#include <gtsam/slam/EssentialMatrixConstraint.h>

#include <gtsam_unstable/geometry/Similarity3.h>

#include "vertigo/betweenFactorSwitchable.h"
#include "vertigo/switchVariableLinear.h"

#include "gtsamTracker.h"

namespace SFO {
    GtsamTracker::GtsamTracker(const std::string &strSettingsFile) {

        loadCameraMatrix(strSettingsFile);

        mPoseId = 0;
        mSwitchId = 0;
        mnLoopsDetected = 0;

        gtsam::Pose3 initialPoseEstimate = gtsam::Pose3();
        mNewValues.insert(gtsam::Symbol('x', mPoseId), initialPoseEstimate);

        gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << gtsam::Vector3(0.2, 0.2, 0.2), gtsam::Vector3(1.0, 1.0, 1.0)).finished()); // Assuming 0.2 rad in roll, pitch, yaw
        //mNewFactors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(gtsam::Symbol('x', mPoseId), initialPoseEstimate, priorPoseNoise);
        mNewFactors.emplace_shared<gtsam::NonlinearEquality<gtsam::Pose3> >(gtsam::Symbol('x',mPoseId), initialPoseEstimate);

        mPoseId++;

        mOdometryNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << gtsam::Vector3::Constant(0.1), gtsam::Vector3::Constant(0.05)).finished()); // 10cm std on x,y,z 0.05 rad on roll,pitch,yaw

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

    void GtsamTracker::update(const libviso2::Matrix &T_delta,
                              const std::vector<libviso2::Matcher::p_match> &vMatches,
                              const std::vector<int32_t> &vInliers) {


        gtsam::Pose3 pose3T_delta = cvtMatrix2Pose3(T_delta);
        gtsam::Pose3 lastPose;
        lastPose = mIsam.calculateEstimate<gtsam::Pose3>(gtsam::Symbol('x', mPoseId-1));
        mNewValues.insert(gtsam::Symbol('x', mPoseId), lastPose * pose3T_delta);


        gtsam::BetweenFactor<gtsam::Pose3> betweenFactor(gtsam::Symbol('x', mPoseId-1), gtsam::Symbol('x', mPoseId), pose3T_delta, mOdometryNoise);
        mNewFactors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(betweenFactor);


        mPoseId++;

    }

    std::vector<libviso2::Matrix> GtsamTracker::optimize() {


        mIsam.update(mNewFactors, mNewValues);
        //mIsam.update();
        mCurrentEstimate = mIsam.calculateEstimate();
        mNewFactors.resize(0);
        mNewValues.clear();


        std::vector<libviso2::Matrix> poses;
        for(size_t i = 0; i < mPoseId; i++) {
            gtsam::Pose3 tempPose = mCurrentEstimate.at<gtsam::Pose3>(gtsam::Symbol('x', i));
            poses.push_back(cvtPose32Matrix(tempPose));
        }
        return poses;

    }


    gtsam::Pose3 GtsamTracker::cvtMatrix2Pose3(const libviso2::Matrix &Tin) {
        auto R = gtsam::Rot3(Tin.val[0][0], Tin.val[0][1], Tin.val[0][2],
                        Tin.val[1][0], Tin.val[1][1], Tin.val[1][2],
                        Tin.val[2][0], Tin.val[2][1], Tin.val[2][2]);
        auto t = gtsam::Point3(Tin.val[0][3], Tin.val[1][3], Tin.val[2][3]);

        gtsam::Pose3 Tout(R, t);
        return Tout;
    }

    gtsam::Rot3 GtsamTracker::cvtMatrix2Rot3(const cv::Mat &Rin) {
        if(Rin.rows != 3 || Rin.cols != 3) {
            std::cerr << "Error! Rotation matrix must be 3x3" << std::endl;
            return gtsam::Rot3();
        }
        auto Rout = gtsam::Rot3(Rin.at<double>(0, 0), Rin.at<double>(0, 1), Rin.at<double>(0, 2),
                                Rin.at<double>(1, 0), Rin.at<double>(1, 1), Rin.at<double>(1, 2),
                                Rin.at<double>(2, 0), Rin.at<double>(2, 1), Rin.at<double>(2, 2));
        return Rout;
    }

    gtsam::Point3 GtsamTracker::cvtMatrix2Point3(const cv::Mat &tin) {
        if(tin.rows >= tin.cols) {
            if(tin.rows != 3 || tin.cols != 1) {
                std::cerr << "Error! Translation vector must be 3x1 or 1x3" << std::endl;
                return gtsam::Point3();
            }
            auto tout = gtsam::Point3(tin.at<double>(0, 0), tin.at<double>(1, 0), tin.at<double>(2, 0));
            return tout;
        } else {
            if(tin.rows != 1 || tin.cols != 3) {
                std::cerr << "Error! Translation vector must be 3x1 or 1x3" << std::endl;
                return gtsam::Point3();
            }
            auto tout = gtsam::Point3(tin.at<double>(0, 0), tin.at<double>(0, 1), tin.at<double>(0, 2));
            return tout;
        }
    }

    gtsam::Unit3 GtsamTracker::cvtMatrix2Unit3(const cv::Mat &tin) {
        if(tin.rows >= tin.cols) {
            if(tin.rows != 3 || tin.cols != 1) {
                std::cerr << "Error! Translation vector must be 3x1 or 1x3" << std::endl;
                return gtsam::Unit3();
            }
            auto tout = gtsam::Unit3(tin.at<double>(0, 0), tin.at<double>(1, 0), tin.at<double>(2, 0));
            return tout;
        } else {
            if(tin.rows != 1 || tin.cols != 3) {
                std::cerr << "Error! Translation vector must be 3x1 or 1x3" << std::endl;
                return gtsam::Unit3();
            }
            auto tout = gtsam::Unit3(tin.at<double>(0, 0), tin.at<double>(0, 1), tin.at<double>(0, 2));
            return tout;
        }
    }

    libviso2::Matrix GtsamTracker::cvtPose32Matrix(const gtsam::Pose3 &Min) {
        libviso2::Matrix Mout(4, 4);
        for (int i = 0; i < Min.matrix().rows(); i++){
            for (int j = 0; j < Min.matrix().cols(); j++){
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
