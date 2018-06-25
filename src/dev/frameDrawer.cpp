#include "frameDrawer.h"

namespace SFO {
    FrameDrawer::FrameDrawer() {
        mIm = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
    }

    cv::Mat FrameDrawer::drawFrame() {
        cv::Mat im;

    }
} // namspace SFO