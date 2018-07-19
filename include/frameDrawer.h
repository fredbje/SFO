
#ifndef SFO_FRAMEDRAWER_H
#define SFO_FRAMEDRAWER_H

#include "libviso2/matrix.h"
#include "libviso2/viso_stereo.h"
#include <opencv2/core/core.hpp>
#include <mutex>

namespace SFO {
    class FrameDrawer {
    public:
        FrameDrawer(const std::string &strSettingsFile);
        ~FrameDrawer();

        void update(const cv::Mat &imgLeft,
                    const cv::Mat &imgRight,
                    const std::vector<libviso2::Matcher::p_match> &vMatches,
                    const std::vector<int32_t> &vInliers);

        void run();

        void requestFinish();

    private:
        void drawFrame();
        void drawText();

        std::mutex mMutexFinish;
        bool mbFinishRequested = false;
        bool checkFinish();

        int mWidth;
        int mHeight;

        cv::Mat mImgDisplay;
        cv::Mat mImgDisplayUpper;
        cv::Mat mImgDisplayLower;

        // Display time for each image (1/fps)
        float mT;

        std::mutex mMutex;

        std::vector<libviso2::Matcher::p_match> mvMatches;
        std::vector<int32_t> mvInliers;
        size_t mnMatches;
        size_t mnInliers;

    };
} // namespace SFO
#endif //SFO_FRAMEDRAWER_H