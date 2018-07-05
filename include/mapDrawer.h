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

    class MapDrawer{
    public:

        MapDrawer(const std::string &strSettingsFile);

        MapDrawer(const std::string &strSettingsFile, const std::vector<libviso2::Matrix> &vGtPoses);

        ~MapDrawer();

        void updateGtsamPoses(std::vector<libviso2::Matrix> *pvGtsamPoses);
        void updateLibviso2Poses(std::vector<libviso2::Matrix> *pvLibviso2Poses);

        void start();

        void requestFinish();

    private:
        libviso2::Matrix mPose;
        std::vector<libviso2::Matrix> *mpvGtsamPoses;
        std::vector<libviso2::Matrix> *mpvLibviso2Poses;
        std::vector<libviso2::Matrix> *mpvGtPoses;

        enum Color { red, green, blue };
        void drawCamera(pangolin::OpenGlMatrix &Twc, Color color);
        void drawLines(pangolin::OpenGlMatrix T1, pangolin::OpenGlMatrix T2, Color color);

        // Return global translation matrix
        pangolin::OpenGlMatrix getOpenGlMatrix(libviso2::Matrix pose);

        std::mutex mMutexGtsamPoses;
        std::mutex mMutexLibviso2Poses;

        std::mutex mMutexFinish;
        bool checkFinish();
        bool mbFinishRequested = false;

        double mViewpointX, mViewpointY, mViewpointZ, mViewpointF;





    };

}


#endif //SFO_DRAWER_H