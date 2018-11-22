#include<iostream>

#include "libviso2/matrix.h"
#include "libviso2/viso_stereo.h"

#include "gtsamTracker.h"
#include "mapDrawer.h"
#include "frameDrawer.h"
#include "system.h"
#include "loadFunctions.h"
#include "oxts.h"

int main(int argc, char **argv){
    if(argc != 2)
    {
        std::cerr << std::endl << "Usage: ./ffi_stereo path_to_settings" << std::endl;
        return 1;
    }

    std::string strSettingsFile = argv[1];
    cv::FileStorage fSettings(strSettingsFile, cv::FileStorage::READ);
    if (!fSettings.isOpened()) {
        std::cerr << "Failed to open settings file at: " << strSettingsFile
                  << " in ffi-stereo.cpp" << std::endl;
        return 1;
    }

    std::string strSequenceDir = fSettings["sequenceDir"];
    std::string strVocabularyFile = fSettings["vocabularyFile"];
    
    fSettings.release();

    std::vector<std::string> vstrLeftImages;
    std::vector<std::string> vstrRightImages;
    loadImageFileNames(strSequenceDir, vstrLeftImages, vstrRightImages);


    SFO::System SLAM(strSettingsFile, strVocabularyFile);
    cv::Mat imgLeft;
    cv::Mat imgRight;

    for (std::size_t i = 0; i < vstrLeftImages.size(); i++) { // 4541 images
        loadImages(imgLeft, imgRight, vstrLeftImages[i], vstrRightImages[i]);
        SLAM.trackStereo(imgLeft, imgRight);
    }

    SLAM.shutdown();

    std::cout << "SFO_main complete! Exiting ..." << std::endl;

	return 0;
}