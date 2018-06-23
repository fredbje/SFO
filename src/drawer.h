#ifndef SFO_DRAWER_H
#define SFO_DRAWER_H

#include <string>
#include <iostream>
#include <Eigen/Eigen>
#include <boost/thread.hpp>

#include "libviso2/matrix.h"

#include <Eigen/Core>
#include <Eigen/Dense>

// For drawing
#include <pangolin/pangolin.h>

namespace SFO {

    class Drawer{
    public:

        // Default constructor
        Drawer();

        Drawer(std::string gtPosesFileName);

        ~Drawer();

        // Updates the local copy of poses
        void updatePoses(std::vector<libviso2::Matrix>* gtsamPoseVec);
        void updatePoseslib(std::vector<libviso2::Matrix>* libviso2PoseVec);

        // Start the pangolin viewer
        // Will continue to run, till it is exited
        void start();

    private:
        libviso2::Matrix mPose;
        std::vector<libviso2::Matrix>* mptrPose;
        std::vector<libviso2::Matrix>* mptrPoselib;
        std::vector<libviso2::Matrix>* mptrPoseTruth;
        // features

        // Private function to draw current frame
        // Taken from orb_slam2
        // https://github.com/raulmur/ORB_SLAM2/blob/master/src/MapDrawer.cc#L179-L219
        void drawCurrentCamera(pangolin::OpenGlMatrix &Twc);
        void drawCurrentCameraRed(pangolin::OpenGlMatrix &Twc);
        void drawCurrentCameraBlue(pangolin::OpenGlMatrix &Twc);

        // Return global translation matrix
        pangolin::OpenGlMatrix get_matrix(libviso2::Matrix pose);

    };

}


#endif //SFO_DRAWER_H