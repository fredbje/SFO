#ifndef SFO_DRAWER_H
#define SFO_DRAWER_H

#include <string>
#include <iostream>
#include <Eigen/Eigen>
#include <boost/thread.hpp>

#include "libviso2/matrix.h"
#include "libviso2/viso_stereo.h"

#include <opencv2/core/mat.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>

// For drawing
#include <pangolin/pangolin.h>

namespace SFO {

    class Drawer{
    public:

        // Default constructor
        Drawer(libviso2::VisualOdometryStereo *pViso, const std::string &strSettingPath);

        //Drawer(std::string gtPosesFileName);

        ~Drawer();

        void setCurrentCameraPose(const cv::Mat &Tcw);

        cv::Mat drawFrame();

        void getCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M);

        // Updates the local copy of poses
        void updatePoses(std::vector<libviso2::Matrix> *gtsamPoseVec);
        void updatePoseslib(std::vector<libviso2::Matrix> *libviso2PoseVec);

        // Start the pangolin viewer
        // Will continue to run, till it is exited
        void start();

        void drawCurrentCamera(pangolin::OpenGlMatrix &Twc);
        void drawCurrentCameraRed(pangolin::OpenGlMatrix &Twc);
        void drawCurrentCameraBlue(pangolin::OpenGlMatrix &Twc);

        void updateImages(const cv::Mat &imLeft, const cv::Mat &imRight);

    private:
        libviso2::Matrix mPose;
        libviso2::VisualOdometryStereo *mpViso;

        std::vector<libviso2::Matrix>* mpvPoseGtsam;
        std::vector<libviso2::Matrix>* mpvPoseLibviso2;
        std::vector<libviso2::Matrix>* mpvPoseGT;
        // features

        cv::Mat mImLeft, mImRight;

        // Return global translation matrix
        pangolin::OpenGlMatrix get_matrix(libviso2::Matrix pose);

        float mKeyFrameSize;
        float mKeyFrameLineWidth;
        float mGraphLineWidth;
        float mPointSize;
        float mCameraSize;
        float mCameraLineWidth;

        cv::Mat mCameraPose;
        std::mutex mMutexCamera;
    };

}


#endif //SFO_DRAWER_H