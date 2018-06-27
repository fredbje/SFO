#include "frameDrawer.h"
#include <opencv2/imgproc/imgproc.hpp> // cvtcolor
#include <iomanip>

namespace SFO {
    FrameDrawer::FrameDrawer(libviso2::VisualOdometryStereo * const pViso, const cv::Size &szImgSize)
            : mpViso(pViso) {
        mImgDisplay = cv::Mat(2*szImgSize.height + 5, szImgSize.width, CV_8UC3);
        mImgDisplayUpper = cv::Mat(mImgDisplay, cv::Rect(0, 0, szImgSize.width, szImgSize.height));
        mImgDisplayLower = cv::Mat(mImgDisplay, cv::Rect(0, szImgSize.height + 5, szImgSize.width, szImgSize.height));
    }
    
    void FrameDrawer::update(const cv::Mat &imgLeft, const cv::Mat &imgRight) {
        std::unique_lock<std::mutex> lock(mMutexUpdate);
        cv::cvtColor(imgLeft, mImgDisplayUpper, CV_GRAY2RGB);
        cv::cvtColor(imgRight, mImgDisplayLower, CV_GRAY2RGB);
        mvMatches = mpViso->getMatches();
        mvInliers = mpViso->getInlierIndices();
        mnMatches = mpViso->getNumberOfMatches();
        mnInliers = mpViso->getNumberOfInliers();
    }
    
    const cv::Mat FrameDrawer::drawFrame() {
        std::cout << ", Matches: " << std::fixed << std::setprecision(0) << mnMatches;
        std::cout << ", Inliers: " << std::fixed << std::setprecision(2)
                  << 100.0*mnInliers/mnMatches << " %" << std::endl;

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
        return mImgDisplay;
    }
    
} // namespace SFO