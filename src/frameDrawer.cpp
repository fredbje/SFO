#include "frameDrawer.h"
#include <opencv2/imgproc/imgproc.hpp> // cvtcolor
#include <iomanip>

namespace SFO {
    FrameDrawer::FrameDrawer(libviso2::VisualOdometryStereo *pTracker, const cv::Size &szImgSize)
            : mpTracker(pTracker) {
        mszImgSize = szImgSize;
        int textHeight = 20; // Actually 10, but leaving som space over and under
        int borderHeight = 5; // Border between images
        mImgDisplay = cv::Mat(2*szImgSize.height + borderHeight + textHeight, szImgSize.width, CV_8UC3);
        mImgDisplayUpper = cv::Mat(mImgDisplay, cv::Rect(0, 0, szImgSize.width, szImgSize.height));
        mImgDisplayLower = cv::Mat(mImgDisplay, cv::Rect(0, szImgSize.height + borderHeight, szImgSize.width, szImgSize.height));
    }

    void FrameDrawer::update(const cv::Mat &imgLeft, const cv::Mat &imgRight) {
        std::unique_lock<std::mutex> lock(mMutexUpdate);
        cv::cvtColor(imgLeft, mImgDisplayUpper, CV_GRAY2RGB);
        cv::cvtColor(imgRight, mImgDisplayLower, CV_GRAY2RGB);
        mvMatches = mpTracker->getMatches();
        mvInliers = mpTracker->getInlierIndices();
        mnMatches = mpTracker->getNumberOfMatches();
        mnInliers = mpTracker->getNumberOfInliers();
    }

    const cv::Mat FrameDrawer::drawFrame() {
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
        return mImgDisplay;
    }

    void FrameDrawer::drawText() {
        std::stringstream s;
        s << "Matches: " << std::fixed << std::setprecision(0) << mnMatches << ", Inliers: "
                << std::fixed << std::setprecision(2) << 100.0*mnInliers/mnMatches << "%";
        mImgDisplay.rowRange(2*mszImgSize.height + 5, 2*mszImgSize.height + 25) = cv::Mat::zeros(20,mImgDisplay.cols,mImgDisplay.type());
        cv::putText(mImgDisplay, s.str(), cv::Point(5,mImgDisplay.rows-5),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,255,255),1,8);
    }

} // namespace SFO