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
#include <gtsam/slam/SmartProjectionPoseFactor.h>
#include <gtsam/geometry/EssentialMatrix.h>
#include <gtsam/slam/EssentialMatrixConstraint.h>

#include <gtsam_unstable/geometry/Similarity3.h>

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
        mSwitchId = 0;

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

    void GtsamTracker::update(const libviso2::Matrix &T_delta,
                              const std::vector<libviso2::Matcher::p_match> &vMatches,
                              const std::vector<int32_t> &vInliers,
                              const oxts &navdata,
                              const DLoopDetector::DetectionResult &loopResult) {


        gtsam::Pose3 pose3T_delta = cvtMatrix2Pose3(T_delta);
        gtsam::Pose3 lastPose;
        lastPose = mIsam.calculateEstimate<gtsam::Pose3>(gtsam::Symbol('x', mPoseId-1));
        mNewValues.insert(gtsam::Symbol('x', mPoseId), lastPose * pose3T_delta);

        // If loop, add smart factors between matched frames
        if(false)//loopResult.detection()) {
            cv::Mat E = cv::findEssentialMat(loopResult.queryFeatures, loopResult.matchFeatures, mK1->fx(), cv::Point2d(mK1->px(), mK1->py()));
            cv::Mat Rtmp, ttmp;
            recoverPose(E, loopResult.queryFeatures, loopResult.matchFeatures, Rtmp, ttmp, mK1->fx(), cv::Point2d(mK1->px(), mK1->py()));
            gtsam::Rot3 R = cvtMatrix2Rot3(Rtmp);
            gtsam::Point3 t = cvtMatrix2Point3(ttmp);

            //gtsam::EssentialMatrix trueE(trueRotation, trueDirection);
            //mNewFactors.emplace_shared<gtsam::EssentialMatrixConstraint>()

            gtsam::Pose3 T = gtsam::Pose3(R, t);
            gtsam::noiseModel::Diagonal::shared_ptr loopNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << gtsam::Vector3::Constant(0.2), gtsam::Vector3(5, 1, 1)).finished());

            /*
            Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity();
            Eigen::Matrix<double, 6, 6> mgtsam = Eigen::Matrix<double, 6, 6>::Identity();
            mgtsam.block(0,0,3,3) = information.block(3,3,3,3); // cov rotation
            mgtsam.block(3,3,3,3) = information.block(0,0,3,3); // cov translation
            mgtsam.block(0,3,3,3) = information.block(0,3,3,3); // off diagonal
            mgtsam.block(3,0,3,3) = information.block(3,0,3,3); // off diagonal
            gtsam::SharedNoiseModel model = gtsam::noiseModel::Gaussian::Information(mgtsam);
            */

            //mNewFactors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(gtsam::Symbol('x', loopResult.query), gtsam::Symbol('x', loopResult.match), T, loopNoise);

            double switchPrior = 1.0;
            mNewValues.insert(gtsam::Symbol('s', mSwitchId), vertigo::SwitchVariableLinear(switchPrior));
            gtsam::noiseModel::Diagonal::shared_ptr switchPriorModel = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector1(1.0));
            mNewFactors.add(gtsam::PriorFactor<vertigo::SwitchVariableLinear>(gtsam::Symbol('s', mSwitchId), vertigo::SwitchVariableLinear(switchPrior), switchPriorModel));

            mNewFactors.add(vertigo::BetweenFactorSwitchableLinear<gtsam::Pose3>(gtsam::Symbol('x', loopResult.query), gtsam::Symbol('x', loopResult.match), gtsam::Symbol('s', mSwitchId++), T, loopNoise));

            //gtsam::Similarity3 simT = gtsam::Similarity3(R, t, 1.0);
            //gtsam::noiseModel::Diagonal::shared_ptr simNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(7) << gtsam::Vector6::Constant(0.1), 10).finished()); // 10cm std on x,y,z 0.05 rad on roll,pitch,yaw
            //mNewFactors.emplace_shared<gtsam::BetweenFactor<gtsam::Similarity3> >(gtsam::Symbol('x', loopResult.query), gtsam::Symbol('x', loopResult.match), simT, simNoise);

            /*
            const gtsam::noiseModel::Isotropic::shared_ptr imageNoiseModel = gtsam::noiseModel::Isotropic::Sigma(2, 1.0);
            gtsam::Point2 pt1, pt2;
            for(size_t kk = 0; kk < loopResult.inliers.size(); kk++) {




                if(static_cast<unsigned>(loopResult.inliers[kk] == 0)) {
                    continue;
                }
                pt1 = gtsam::Point2(loopResult.matchFeatures.at<float>(kk, 0), loopResult.matchFeatures.at<float>(kk, 1));
                pt2 = gtsam::Point2(loopResult.queryFeatures.at<float>(kk, 0), loopResult.queryFeatures.at<float>(kk, 1));
                gtsam::SmartProjectionPoseFactor<gtsam::Cal3_S2>::shared_ptr smartFactor(new gtsam::SmartProjectionPoseFactor<gtsam::Cal3_S2>(imageNoiseModel, mK1));
                smartFactor->add(pt1, gtsam::Symbol('x', loopResult.match));
                smartFactor->add(pt2, gtsam::Symbol('x', loopResult.query));
                mNewFactors.push_back(smartFactor);

            }
                 */
        }

/*
        const gtsam::noiseModel::Isotropic::shared_ptr imageNoiseModel = gtsam::noiseModel::Isotropic::Sigma(3, 1); // 1 pixel in ul, ur, v
        gtsam::SmartStereoProjectionPoseFactor::shared_ptr smartFactor(new gtsam::SmartStereoProjectionPoseFactor(imageNoiseModel));
        gtsam::StereoPoint2 spt1, spt2;
        for(size_t kk = 0; kk < vInliers.size(); kk++) {
            getMatchedPairs(vMatches[vInliers[kk]], spt1, spt2);
            mNewFactors.push_back(smartFactor);
            smartFactor = gtsam::SmartStereoProjectionPoseFactor::shared_ptr(new gtsam::SmartStereoProjectionPoseFactor(imageNoiseModel));
            smartFactor->add(spt1, gtsam::Symbol('x', mPoseId-1), mK);
            smartFactor->add(spt2, gtsam::Symbol('x', mPoseId), mK);
        }
*/

/*
        const gtsam::noiseModel::Isotropic::shared_ptr imageNoiseModel = gtsam::noiseModel::Isotropic::Sigma(2, 1.0);
        gtsam::Point2 pt1, pt2;
        for(size_t kk = 0; kk < vInliers.size(); kk++) {
            const libviso2::Matcher::p_match match = vMatches[vInliers[kk]];
            pt1 = gtsam::Point2(match.u1p, match.v1p);
            pt2 = gtsam::Point2(match.u1c, match.v1c);
            gtsam::SmartProjectionPoseFactor<gtsam::Cal3_S2>::shared_ptr smartFactor(new gtsam::SmartProjectionPoseFactor<gtsam::Cal3_S2>(imageNoiseModel, mK1));
            smartFactor->add(pt1, gtsam::Symbol('x', mPoseId-1));
            smartFactor->add(pt2, gtsam::Symbol('x', mPoseId));
            mNewFactors.push_back(smartFactor);
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


        mNewFactors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3> >(gtsam::Symbol('x', mPoseId-1), gtsam::Symbol('x', mPoseId), pose3T_delta, mOdometryNoise);
        //mStereoCamera = gtsam::StereoCamera(gtsamPose, mK);


        mPoseId++;
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

    /*
    gtsam::Rot3 GtsamTracker::cvtMatrix2Rot3(const libviso2::Matrix &Rin) {
        auto Rout = gtsam::Rot3(Rin.val[0][0], Rin.val[0][1], Rin.val[0][2],
                             Rin.val[1][0], Rin.val[1][1], Rin.val[1][2],
                             Rin.val[2][0], Rin.val[2][1], Rin.val[2][2]);
        return Rout;
    }

    gtsam::Point3 GtsamTracker::cvtMatrix2Point3(const libviso2::Matrix &tin) {
        auto tout = gtsam::Point3(tin.val[0][0], tin.val[1][0], tin.val[2][0]);
    }
    */

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
