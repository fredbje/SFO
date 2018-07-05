#include <gtsam/nonlinear/Marginals.h>
#include <opencv2/core/persistence.hpp>
#include "gtsamTracker.h"

namespace SFO {
    GtsamTracker::GtsamTracker(const std::string &strSettingsFile) {
        loadCameraMatrix(strSettingsFile);
        mMeasurementNoise = gtsam::noiseModel::Isotropic::Sigma(2, 1); // 1 pixel in u and v
        gtsam::Pose3 initialPose; // Origin is default constructor
        mStereoCamera = gtsam::StereoCamera(initialPose, mK);
        mPoseId = 0;
        mLandmarkId = 1;
        mInitialEstimate.insert(gtsam::Symbol('x', mPoseId++), mStereoCamera.pose());
    }

    GtsamTracker::~GtsamTracker() = default;


    void GtsamTracker::getMatchedPairs(const libviso2::Matcher::p_match &match,
                      gtsam::StereoPoint2 &spt1,
                      gtsam::StereoPoint2 &spt2){
        spt1 = gtsam::StereoPoint2(match.u1p, match.u2p, match.v1p);
        spt2 = gtsam::StereoPoint2(match.u1c, match.u2c, match.v1c);
    }

    void GtsamTracker::update(libviso2::Matrix pose, const std::vector<libviso2::Matcher::p_match> &vMatches,
                              const std::vector<int32_t> &vInliers) {
        gtsam::Matrix gtsamPose;
        cvtMatrix2Gtsam(pose, gtsamPose);
        mInitialEstimate.insert(gtsam::Symbol('x', mPoseId), gtsam::Pose3(gtsamPose));

        gtsam::StereoPoint2 spt1, spt2;
        for(size_t kk=0; kk<vInliers.size()-1; kk++) { // TODO remove -1?
            getMatchedPairs(vMatches[vInliers[kk]], spt1, spt2);
            // TODO find id of landmark. Landmarks need a global id.

            if()
            mGraph.emplace_shared<
                    gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3>>(spt1, mMeasurementNoise,
                                                                             gtsam::Symbol('x', mPoseId - 1),
                                                                             gtsam::Symbol('l', mLandmarkId), mK);
            mGraph.emplace_shared<
                    gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3>>(spt2, mMeasurementNoise,
                                                                             gtsam::Symbol('x', mPoseId),
                                                                             gtsam::Symbol('l', mLandmarkId++), mK);






            gtsam::Point3 pt = stereoCam.backproject(spt1);
            if (pt.z() <= 40 * (mK->baseline())) {}




            Point2 measurement = camera.project(points[j]);
            graph.emplace_shared<GenericProjectionFactor<Pose3, Point3, Cal3_S2>>(measurement, measurementNoise, Symbol('x', i), Symbol('l', j), K);

            if (!mInitialEstimate.exists(gtsam::Symbol('l'), l)) {
                worldPoint = backProject();
                mInitialEstimate.insert(gtsam::Symbol('l', worldPoint));
            }
        }
        mPoseId++;
    }

    void GtsamTracker::optimize(std::vector<libviso2::Matrix> *gtsamPoses) {


        //setup the first pose
        //assume the first pose is the origin
        gtsam::Pose3 firstPose;
        mGraph.push_back(gtsam::NonlinearEquality<gtsam::Pose3>(gtsam::Symbol('x', 1),firstPose));
        mInitialEstimate.insert(gtsam::Symbol('x',1), firstPose);

        //get the second camera pose and initialize the second pose
        gtsam::Rot3 R_2to1;
        gtsam::Point3 T_2in1;
        cvtMatrix2RT(pose,R_2to1,T_2in1);
        mInitialEstimate.insert(gtsam::Symbol('x',2),gtsam::Pose3(R_2to1,T_2in1));

        //setup the stereo camera
        gtsam::StereoCamera stereoCam(firstPose, mK);


        size_t counter = 0;
        //construct the points
        for(size_t kk=0; kk<vInliers.size()-1; kk++){
            gtsam::StereoPoint2 spt1 = gtsam::StereoPoint2(vMatches[vInliers[kk]].u1p,vMatches[vInliers[kk]].u2p,vMatches[vInliers[kk]].v1p);
            gtsam::StereoPoint2 spt2 = gtsam::StereoPoint2(vMatches[vInliers[kk]].u1c,vMatches[vInliers[kk]].u2c,vMatches[vInliers[kk]].v1c);

            gtsam::Point3 pt = stereoCam.backproject(spt1);
            if(pt.z() <= 40*(mK->baseline())) {

                counter++;
                mInitialEstimate.insert(gtsam::Symbol('f', counter), pt);
                mGraph.push_back(
                        gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3>(spt1, mNoiseModel, gtsam::Symbol('x', 1),
                                                                                gtsam::Symbol('f', counter), mK));
                graph.push_back(
                        gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3>(spt2, mNoiseModel, gtsam::Symbol('x', 2),
                                                                                gtsam::Symbol('f', counter), K));
            }
        }

        //finish the graph and begin to optimization
        gtsam::LevenbergMarquardtParams lmParams;
        lmParams.maxIterations = 10;
        gtsam::LevenbergMarquardtOptimizer optimizer(mGraph, mInitialEstimate, lmParams);
        gtsam::Values result = optimizer.optimize();
        gtsam::Pose3 tempPose = result.at<gtsam::Pose3>(gtsam::Symbol('x',2));
        cvtgtPose2RT(tempPose,poseOpt);

    }

    void GtsamTracker::cvtMatrix2Gtsam(const libviso2::Matrix &Min, gtsam::Matrix &Mout) {
        long m = Min.m;
        long n = Min.n;

        Mout.resize(m,n);
        for (int i=0;i<m; i++){
            for (int j=0;j<n;j++){
                Mout(i,j) = Min.val[i][j];
            }
        }
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
    }
}
