#ifndef SFO_SYSTEM_H
#define SFO_SYSTEM_H

#include <thread>

#include "libviso2/matrix.h"
#include "libviso2/viso_stereo.h"
#include "viewer.h"
#include "drawer.h"

namespace SFO {
	class System {
	public:
		// Initialize the SLAM system. It launches the Viewer thread
		System(const std::string &strSettingsFile, const bool bUseViewer = true);

		~System();

		void trackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp);

		void shutdown();

    private:
		libviso2::VisualOdometryStereo::parameters mParams;
		libviso2::VisualOdometryStereo *mpViso;

		Viewer *mpViewer;
		std::thread *mptViewer;

		Drawer *mpDrawer;

        libviso2::Matrix mLibviso2Pose;
        std::vector<libviso2::Matrix> *mpvLibviso2Poses;

	};
} // namespace SFO

#endif