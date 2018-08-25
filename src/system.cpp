#include <thread>
#include <opencv/cv.hpp>

#include "gtsamTracker.h"
#include "mapDrawer.h"
#include "frameDrawer.h"
#include "system.h"

namespace SFO {
    System::System(const std::string &strSettingsFile,
                   const std::string &strVocabularyFile,
                   const oxts &navdata0,
                   const libviso2::Matrix &imu_T_cam) {
        mnFrame = 0;
        loadSettings(strSettingsFile);
        mpLoopDetector = new LoopDetector(strVocabularyFile, strSettingsFile);
        mpTracker = new libviso2::VisualOdometryStereo(mParam);
        mpFrameDrawer = new FrameDrawer(strSettingsFile);
        mtFrameDrawer = std::thread(&FrameDrawer::run, mpFrameDrawer);
        mpGtsamTracker = new GtsamTracker(strSettingsFile, navdata0, imu_T_cam);
        mpMapDrawer = new MapDrawer(strSettingsFile);
        mtMapDrawer = std::thread(&MapDrawer::run, mpMapDrawer);

        // Initialize trajectory to the origin of the IMU.
        mPose = imu_T_cam;
        mpvPoses = new std::vector<libviso2::Matrix>();
        mpvPoses->push_back(imu_T_cam);
        mpMapDrawer->updatePoses(mpvPoses);
    }

    System::System(const std::string &strSettingsFile,
                   const std::string &strVocabularyFile,
                   const oxts &navdata0,
                   const libviso2::Matrix &imu_T_cam,
                   const std::vector<libviso2::Matrix> &vGtPoses)
            : System(strSettingsFile, strVocabularyFile, navdata0, imu_T_cam) {
        mpMapDrawer->setGtPoses(vGtPoses);
    }

    System::~System() {
        std::cout << "System destructor called." << std::endl;
        delete mpTracker;
        delete mpMapDrawer;
        delete mpFrameDrawer;
        delete mpvPoses;
        delete mpLoopDetector;
    }

    void System::trackStereo(const cv::Mat &imgLeft,
                             const cv::Mat &imgRight,
                             const double &timestamp,
                             const oxts &navdata) {
        if(imgLeft.size() != mImgSize || imgRight.size() != mImgSize) {
            std::cerr << "Error, images have different size than specified in settings file." << std::endl;
        }

        DLoopDetector::DetectionResult loopResult;
        std::thread tLoopDetection(&LoopDetector::process, mpLoopDetector, imgLeft, std::ref(loopResult));
        // process new images, push the images back to an internal ring buffer.
        // valid motion estimates are available after calling process for two times.
        // output: returns false if an error occurred.
        bool bTrackOK = mpTracker->process(imgLeft.data, imgRight.data, mDims);
        tLoopDetection.join();

        if (bTrackOK) {
            mvMatches.clear();
            mvInliers.clear();
            // returns previous to current feature matches from internal matcher
            mvMatches = mpTracker->getMatches();
            mvInliers = mpTracker->getInlierIndices();

            // inv(getMotion()) returns {t-1}^T_{t}. Right multiply with last pose to get {0}^T_{t}
            libviso2::Matrix T_delta = libviso2::Matrix::inv(mpTracker->getMotion());
            mpGtsamTracker->update(T_delta, mvMatches, mvInliers, navdata);
            *mpvPoses = mpGtsamTracker->optimize();

            //mPose = mPose * T_delta;
            //mpvPoses->push_back(mPose);

            mpMapDrawer->updatePoses(mpvPoses);

            mpFrameDrawer->update(mnFrame, imgLeft, imgRight, mvMatches, mvInliers, loopResult.detection(), loopResult.match);


        } else if(mnFrame != 0) { // mpTracker->process needs two frames to provide valid motion estimates
            std::cerr << " ... failed!";
        }
        std::cout << std::endl;
        mnFrame++;
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
        std::cout << "Shutting down..." << std::endl;
        mpMapDrawer->requestFinish();
        mpFrameDrawer->requestFinish();
        mpGtsamTracker->save();
        mtMapDrawer.join();
        mtFrameDrawer.join();
    }
} // namespace SFO