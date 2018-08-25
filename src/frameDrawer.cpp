#include "frameDrawer.h"
#include <opencv2/imgproc/imgproc.hpp> // cvtcolor
#include <iomanip> // setprecision
#include <opencv/cv.hpp> // imshow


namespace SFO {
    FrameDrawer::FrameDrawer(const std::string &strSettingsFile) {
        cv::FileStorage fSettings(strSettingsFile, cv::FileStorage::READ);
        if (!fSettings.isOpened()) {
            std::cerr << "Failed to open settings file at: " << strSettingsFile
                      << " in FrameDrawer constructor." << std::endl;
        }
        float fps = fSettings["Camera.fps"];
        mT = (fps == 0) ? 10 : 1/fps;
        mHeight = fSettings["Camera.height"];
        mWidth = fSettings["Camera.width"];

        int textHeight = 20; // Actually 10, but leaving som space over and under
        int borderHeight = 5; // Border between images
        mImgDisplay = cv::Mat(2*mHeight + borderHeight + textHeight, mWidth, CV_8UC3);
        mImgDisplayUpper = cv::Mat(mImgDisplay, cv::Rect(0, 0, mWidth, mHeight));
        mImgDisplayLower = cv::Mat(mImgDisplay, cv::Rect(0, mHeight + borderHeight, mWidth, mHeight));

        mnFrame = 0;
        mnLoops = 0;
        mbLoopDetected = false;
    }

    FrameDrawer::~FrameDrawer() {
        std::cout << "FrameDrawer destructor called." << std::endl;
    }

    void FrameDrawer::update(const unsigned int &nFrame,
                             const cv::Mat &imgLeft,
                             const cv::Mat &imgRight,
                             const std::vector<libviso2::Matcher::p_match> &vMatches,
                             const std::vector<int32_t> &vInliers,
                             const bool &bLoopDetected,
                             const unsigned int &nLoopMatch) {
        std::unique_lock<std::mutex> lock(mMutex);
        mnFrame = nFrame;
        cv::cvtColor(imgLeft, mImgDisplayUpper, CV_GRAY2RGB);
        cv::cvtColor(imgRight, mImgDisplayLower, CV_GRAY2RGB);
        mvMatches = vMatches;
        mvInliers = vInliers;
        mnMatches = vMatches.size();
        mnInliers = vInliers.size();
        mbLoopDetected = bLoopDetected;
        mnLoopMatch = nLoopMatch;
    }

    void FrameDrawer::run() {
        while(true) {
            std::unique_lock<std::mutex> lock(mMutex);
            drawFrame();
            lock.unlock();
            cv::imshow("Stereo Gray Image", mImgDisplay);
            cv::waitKey(static_cast<int>(0.1*1e3)); //mT*1e3
            if(checkFinish()) {
                break;
            }
        }
        std::cout << "Exiting FrameDrawer thread." << std::endl;
    }

    void FrameDrawer::requestFinish() {
        std::unique_lock<std::mutex> lock(mMutexFinish);
        mbFinishRequested = true;
    }

    bool FrameDrawer::checkFinish() {
        std::unique_lock<std::mutex> lock(mMutexFinish);
        return mbFinishRequested;
    }

    void FrameDrawer::drawFrame() {
        for (std::size_t j = 0; j < mvMatches.size(); j++) {
            cv::Scalar color;
            if (std::find(mvInliers.begin(), mvInliers.end(), j) != mvInliers.end()) {
                color = CV_RGB(0, 255, 255);
            } else {
                color = CV_RGB(255, 0, 0);
            }
            libviso2::Matcher::p_match match = mvMatches.at(j);
            cv::Point2f pt_left(match.u1c, match.v1c);
            cv::circle(mImgDisplayUpper, pt_left, 2, color);
            cv::Point2f pt_right(match.u2c, match.v2c);
            cv::circle(mImgDisplayLower, pt_right, 2, color);
        }
        drawText();
    }

    void FrameDrawer::drawText() {
        std::stringstream ss;


        float percentInliers = (mnMatches == 0) ? 0.0f : 100.0f*mnInliers/mnMatches;
        ss << "Frame: " << mnFrame << ", Feature Matches: " << std::fixed << std::setprecision(0) << mnMatches << ", Inliers: "
                << std::fixed << std::setprecision(2) << percentInliers << "%";


        if(mbLoopDetected) {
            ss << ", Loop match " << mnLoopMatch;
            ++mnLoops;
        } else {
            ss << ", No loop match";
        }

        ss << ", Loops in sequence: " << mnLoops;

        mImgDisplay.rowRange(2*mHeight + 5, 2*mHeight + 25) = cv::Mat::zeros(20,mImgDisplay.cols,mImgDisplay.type());
        cv::putText(mImgDisplay, ss.str(), cv::Point(5,mImgDisplay.rows-5), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255,255,255), 1, 8);
    }

} // namespace SFO

