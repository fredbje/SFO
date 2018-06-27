#ifndef SFO_DRAWER_H
#define SFO_DRAWER_H

#include <string>
#include <iostream>
#include <mutex>
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

        Drawer(const std::string &strSettingsFile);

        Drawer(const std::string &strSettingsFile, const std::vector<libviso2::Matrix> &vGtPoses);

        ~Drawer();

        // Updates the local copy of poses
        void updateGtsamPoses(const std::vector<libviso2::Matrix> &pvGtsamPoses);
        void updateLibviso2Poses(const std::vector<libviso2::Matrix> &pvLibviso2Poses);

        // Start the pangolin viewer
        // Will continue to run, till it is exited
        void start();

        void requestFinish();

    private:
        libviso2::Matrix mPose;
        std::vector<libviso2::Matrix> mvGtsamPoses;
        std::vector<libviso2::Matrix> mvLibviso2Poses;
        std::vector<libviso2::Matrix> mvGtPoses;
        // features

        // Private function to draw current frame
        // Taken from orb_slam2
        // https://github.com/raulmur/ORB_SLAM2/blob/master/src/MapDrawer.cc#L179-L219
        void drawCurrentCamera(pangolin::OpenGlMatrix &Twc);
        void drawCurrentCameraRed(pangolin::OpenGlMatrix &Twc);
        void drawCurrentCameraBlue(pangolin::OpenGlMatrix &Twc);

        // Return global translation matrix
        pangolin::OpenGlMatrix getOpenGlMatrix(libviso2::Matrix pose);

        bool checkFinish();
        bool mbFinishRequested = false;

        std::mutex mMutexFinish;
        std::mutex mMutexUpdateGtsam;
        std::mutex mMutexUpdateLibviso2;


        double mViewpointX;
        double mViewpointY;
        double mViewpointZ;
        double mViewpointF;

        float mCameraSize;
        float mCameraLineWidth;
    };

}


#endif //SFO_DRAWER_H