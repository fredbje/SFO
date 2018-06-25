#include "system.h"
#include <opencv2/core/persistence.hpp>

namespace SFO {
	System::System(const std::string &strSettingsFile, const bool bUseViewer) {
		
		std::cout << "Starting SFO" << std::endl;

		cv::FileStorage fSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
		if(!fSettings.isOpened()) {
			std::cerr << "Failed to open settings file at: " << strSettingsFile << std::endl;
		}

        float fx = fSettings["Camera.fx"];
        float fy = fSettings["Camera.fy"];
        float cx = fSettings["Camera.cx"];
        float cy = fSettings["Camera.cy"];

        // calibration parameters for odometery sequence 00
        mParams.calib.f  = 718.85; // focal length in pixels
        mParams.calib.cu = 607.19; // principal point (u-coordinate) in pixels
        mParams.calib.cv = 185.21; // principal point (v-coordinate) in pixels
        mParams.base     = 0.5371; // baseline in meters

        // init visual odometry
        mpViso = new libviso2::VisualOdometryStereo(mParams);

        // init the drawer
        mpDrawer = new Drawer(mpViso, strSettingsFile);

        //Initialize the Viewer thread and launch
        mpViewer = new Viewer(mpDrawer, strSettingsFile);
        mptViewer = new std::thread(&Viewer::run, mpViewer);

        mLibviso2Pose = libviso2::Matrix::eye(4);
        mpvLibviso2Poses = new std::vector<libviso2::Matrix>();
        mpvLibviso2Poses->push_back(mLibviso2Pose);
        mpDrawer->updatePoseslib(mpvLibviso2Poses);

	}

    System::~System() {
      //  delete mpViso;
    }

	void System::trackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp) {

        std::int32_t dims[] = {imLeft.size().width, imLeft.size().height, imLeft.size().width};
        if (mpViso->process(imLeft.data, imRight.data, dims)) {

            mLibviso2Pose = mLibviso2Pose * libviso2::Matrix::inv(mpViso->getMotion());
            mpvLibviso2Poses->push_back(mLibviso2Pose);

            mpDrawer->updatePoseslib(mpvLibviso2Poses);
            mpDrawer->updateImages(imLeft, imRight);

        } else {
            std::cerr << " ... failed!" << std::endl;
        }
	}

	void System::shutdown() {
		if(mpViewer) {
			mpViewer->requestFinish();
			while(!mpViewer->isFinished()) {
				std::this_thread::sleep_for(std::chrono::milliseconds(5));
			}
		}

		if(mpViewer) {
			pangolin::BindToContext("SFO: Map Viewer");
		}
	}

} // namespace SFO