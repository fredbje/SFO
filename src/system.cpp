#include <thread>
#include <opencv/cv.hpp>

#include "libviso2/matrix.h"
#include "libviso2/viso_stereo.h"

#include "gtsamTracker.h"
#include "mapDrawer.h"
#include "frameDrawer.h"
#include "system.h"

namespace SFO {
    System::System(const std::string &strSettingsFile) {
        mFrame = 0;
        loadSettings(strSettingsFile);
        mpTracker = new libviso2::VisualOdometryStereo(mParam);
        mpMapDrawer = new MapDrawer(strSettingsFile);
        mpFrameDrawer = new FrameDrawer(mpTracker, mImgSize);
        mpGtsamTracker = new GtsamTracker(strSettingsFile);
        mtMapDrawer = std::thread(&MapDrawer::start, mpMapDrawer);

        // Initialize trajectory to the origin
        mPose = libviso2::Matrix::eye(4);
        mpvPoses = new std::vector<libviso2::Matrix>();
        mpvPoses->push_back(mPose);
        mpMapDrawer->updatePoses(mpvPoses);
    }

    System::System(const std::string &strSettingsFile, const std::vector<libviso2::Matrix> &vGtPoses) {
        mFrame = 0;
        loadSettings(strSettingsFile);
        mpTracker = new libviso2::VisualOdometryStereo(mParam);
        mpMapDrawer = new MapDrawer(strSettingsFile, vGtPoses);
        mpFrameDrawer = new FrameDrawer(mpTracker, mImgSize);
        mpGtsamTracker = new GtsamTracker(strSettingsFile);
        mtMapDrawer = std::thread(&MapDrawer::start, mpMapDrawer);

        // Initialize trajectory to the origin
        mPose = libviso2::Matrix::eye(4);
        mpvPoses = new std::vector<libviso2::Matrix>();
        mpvPoses->push_back(mPose);
        mpMapDrawer->updatePoses(mpvPoses);
    }

    System::~System() {
        delete mpTracker;
        delete mpMapDrawer;
        delete mpFrameDrawer;
        delete mpvPoses;
    }

    void System::trackStereo(const cv::Mat &imgLeft, const cv::Mat &imgRight, const double &timestamp) {
        std::cout << "Processing: Frame: " << std::setw(4) << mFrame;
        if(imgLeft.size() != mImgSize || imgRight.size() != mImgSize) {
            std::cerr << "Error, images have different size than specified in settings file." << std::endl;
        }

        // process a new images, push the images back to an internal ring buffer.
        // valid motion estimates are available after calling process for two times.
        // output: returns false if an error occurred.
        if (mpTracker->process(imgLeft.data, imgRight.data, mDims)) {
            mvMatches.clear();
            mvInliers.clear();
            // returns previous to current feature matches from internal matcher
            mvMatches = mpTracker->getMatches();
            mvInliers = mpTracker->getInlierIndices();

            // returns transformation from previous to current coordinates as a 4x4
            // homogeneous transformation matrix Tr_delta, with the following semantics:
            // p_t = Tr_delta * p_ {t-1} takes a point in the camera coordinate system
            // at time t_1 and maps it to the camera coordinate system at time t.
            // note: getMotion() returns the last transformation even when process()
            // has failed. this is useful if you wish to linearly extrapolate occasional
            // frames for which no correspondences have been found
            libviso2::Matrix T_delta = mpTracker->getMotion();


            mpGtsamTracker->update(T_delta, mvMatches, mvInliers);
            //mpGtsamTracker->update(mPose, mvMatches, mvInliers);
            *mpvPoses = mpGtsamTracker->optimize();

            //mpvPoses->push_back(mPose);

            mpMapDrawer->updatePoses(mpvPoses);

            mpFrameDrawer->update(imgLeft, imgRight);

            cv::imshow("Stereo Gray Image", mpFrameDrawer->drawFrame());
            cv::waitKey(static_cast<int>(mT*1e3));
        } else if(mFrame != 0) { // mpTracker->process needs two frames to provide valid motion estimates
            std::cerr << " ... failed!";
        }
        std::cout << std::endl;
        mFrame++;
    }

    void System::loadSettings(const std::string &strSettingsFile) {
        cv::FileStorage fSettings(strSettingsFile, cv::FileStorage::READ);

        if(!fSettings.isOpened()) {
            std::cerr << "Failed to open settings file at: " << strSettingsFile << std::endl;
        }

        float fx = fSettings["Camera.fx"]; // focal length in pixels
        float cx = fSettings["Camera.cx"]; // principal point (x/u-coordinate) in pixels
        float cy = fSettings["Camera.cy"]; // principal point (y/v-coordinate) in pixels
        float bf = fSettings["Camera.bf"];
        float base = bf/fx;                // baseline in meters

        mImgSize = cv::Size(fSettings["Camera.width"], fSettings["Camera.height"]);

        mDims[0] = static_cast<std::int32_t>(mImgSize.width);
        mDims[1] = static_cast<std::int32_t>(mImgSize.height);
        mDims[2] = static_cast<std::int32_t>(mImgSize.width);

        mFps = fSettings["Camera.fps"];
        mFps = (mFps < 1) ? 10 : mFps;
        mT = 1/mFps;

        mParam.calib.f  = fx;
        mParam.calib.cu = cx;
        mParam.calib.cv = cy;
        mParam.base     = base;
    }

    void System::shutdown() {
        mpMapDrawer->requestFinish();
        mtMapDrawer.join();
    }
} // namespace SFO