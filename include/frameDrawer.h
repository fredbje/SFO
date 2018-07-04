
#ifndef SFO_FRAMEDRAWER_H
#define SFO_FRAMEDRAWER_H

#include "libviso2/matrix.h"
#include "libviso2/viso_stereo.h"
#include <opencv2/core/core.hpp>
#include <mutex>

namespace SFO {
    class FrameDrawer {
    public:
        FrameDrawer(libviso2::VisualOdometryStereo *pTracker, const cv::Size &szImgSize);

        void update(const cv::Mat &imgLeft, const cv::Mat &imgRight);

        const cv::Mat drawFrame();

    private:
        void drawText();

        cv::Size mszImgSize;

        cv::Mat mImgDisplay;
        cv::Mat mImgDisplayUpper;
        cv::Mat mImgDisplayLower;

        std::mutex mMutexUpdate;

        libviso2::VisualOdometryStereo *mpTracker;

        std::vector<libviso2::Matcher::p_match> mvMatches;
        std::vector<int32_t> mvInliers;
        int mnMatches;
        int mnInliers;

    };
} // namespace SFO
#endif //SFO_FRAMEDRAWER_H