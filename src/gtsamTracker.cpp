#include <gtsam/nonlinear/Marginals.h>
#include "gtsamTracker.h"

namespace SFO {
    GtsamTracker::GtsamTracker() {

    }

    void cvtMatrix2Eigen(const libviso2::Matrix &Min, Eigen::MatrixXd &Mout) {
        long m = Min.m;
        long n = Min.n;

        Mout.resize(m,n);
        for (int i=0;i<m; i++){
            for (int j=0;j<n;j++){
                Mout(i,j) = Min.val[i][j];
            }
        }
    }


    void cvtMatrix2RT(const libviso2::Matrix &Min, gtsam::Rot3 &R, gtsam::Point3 &T) {
        long m = Min.m;
        long n = Min.n;

        if(m!=4 || n!=4){
            std::cerr<<"Not a Transformation Matrix!!"<< std::endl;
        }

        R = gtsam::Rot3(Min.val[0][0], Min.val[0][1],Min.val[0][2],
                        Min.val[1][0], Min.val[1][1],Min.val[1][2],
                        Min.val[2][0], Min.val[2][1],Min.val[2][2]);
        T = gtsam::Point3(Min.val[0][3], Min.val[1][3], Min.val[2][3]);

    }


    void matchedpairs(const libviso2::Matcher::p_match &vMatches,
                      gtsam::StereoPoint2 &p1,
                      gtsam::StereoPoint2 &p2){
        p1 = gtsam::StereoPoint2(vMatches.u1p,vMatches.u2p,vMatches.v1p);
        p2 = gtsam::StereoPoint2(vMatches.u1c,vMatches.u2c,vMatches.v1c);
    }


    void localOptimization(const std::vector<libviso2::Matcher::p_match> &vMatches,
                           const std::vector<int32_t> &vInliers,
                           const libviso2::Matrix &pose, // eye(4)
                           const double sigmaPixel,
                           const gtsam::Cal3_S2Stereo::shared_ptr K,
                           const gtsam::noiseModel::Isotropic::shared_ptr model,
                           libviso2::Matrix &poseOpt) {

        gtsam::NonlinearFactorGraph graph;
        gtsam::Values initialEstimate;

        //setup the first pose
        //assume the first pose is the origin
        gtsam::Pose3 firstPose;
        graph.push_back(gtsam::NonlinearEquality<gtsam::Pose3>(gtsam::Symbol('x', 1),firstPose));
        initialEstimate.insert(gtsam::Symbol('x',1), firstPose);

        //get the second camera pose and initialize the second pose
        gtsam::Rot3 R_2to1;
        gtsam::Point3 T_2in1;
        cvtMatrix2RT(pose,R_2to1,T_2in1);
        initialEstimate.insert(gtsam::Symbol('x',2),gtsam::Pose3(R_2to1,T_2in1));

        //setup the stereo camera
        gtsam::StereoCamera stereoCam(firstPose,K);


        size_t counter = 0;
        //construct the points
        for(size_t kk=0; kk<vInliers.size()-1; kk++){
            gtsam::StereoPoint2 spt1 = gtsam::StereoPoint2(vMatches[vInliers[kk]].u1p,vMatches[vInliers[kk]].u2p,vMatches[vInliers[kk]].v1p);
            gtsam::StereoPoint2 spt2 = gtsam::StereoPoint2(vMatches[vInliers[kk]].u1c,vMatches[vInliers[kk]].u2c,vMatches[vInliers[kk]].v1c);

            gtsam::Point3 pt = stereoCam.backproject(spt1);
            if(pt.z() <= 40*(K->baseline())) {

                counter++;
                initialEstimate.insert(gtsam::Symbol('f', counter), pt);
                graph.push_back(
                        gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3>(spt1, model, gtsam::Symbol('x', 1),
                                                                                gtsam::Symbol('f', counter), K));
                graph.push_back(
                        gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3>(spt2, model, gtsam::Symbol('x', 2),
                                                                                gtsam::Symbol('f', counter), K));
            }
        }

        //finish the graph and begin to optimization
        gtsam::LevenbergMarquardtParams lmParams;
        lmParams.maxIterations = 10;
        gtsam::LevenbergMarquardtOptimizer optimizer(graph, initialEstimate, lmParams);
        gtsam::Values result = optimizer.optimize();
        gtsam::Pose3 tempPose = result.at<gtsam::Pose3>(gtsam::Symbol('x',2));
        cvtgtPose2RT(tempPose,poseOpt);

    }

    void cvtgtPose2RT(const gtsam::Pose3 &pose, libviso2::Matrix &ptrM) {

        ptrM.val[0][0] = pose.matrix()(0,0);
        ptrM.val[1][0] = pose.matrix()(1,0);
        ptrM.val[2][0] = pose.matrix()(2,0);
        ptrM.val[3][0] = pose.matrix()(3,0);
        ptrM.val[0][1] = pose.matrix()(0,1);
        ptrM.val[1][1] = pose.matrix()(1,1);
        ptrM.val[2][1] = pose.matrix()(2,1);
        ptrM.val[3][1] = pose.matrix()(3,1);
        ptrM.val[0][2] = pose.matrix()(0,2);
        ptrM.val[1][2] = pose.matrix()(1,2);
        ptrM.val[2][2] = pose.matrix()(2,2);
        ptrM.val[3][2] = pose.matrix()(3,2);
        ptrM.val[0][3] = pose.matrix()(0,3);
        ptrM.val[1][3] = pose.matrix()(1,3);
        ptrM.val[2][3] = pose.matrix()(2,3);
        ptrM.val[3][3] = pose.matrix()(3,3);

    }
}
