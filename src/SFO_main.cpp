#include<iostream>
#include<iomanip>

#include<Eigen/Core>

#include <boost/lambda/lambda.hpp>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>

#include <png++/png.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
//#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "libviso2/matrix.h"
#include "libviso2/viso_stereo.h"

//#include "GeographicLib/Config.h"

#include "stereo.h"
#include "drawer.h"

int main(){
    std::string sequenceDirPath = "/home/fbjerkaas/Master/Datasets/kitti-gray/sequences/00/";
    std::string gtPosesFileName = "/home/fbjerkaas/Master/Datasets/kitti-poses/dataset/poses/00.txt";

    std::cout << "Path to image sequence: " << sequenceDirPath << std::endl;
    std::cout << "Path to ground truth poses: " << gtPosesFileName << std::endl;

    size_t width = 1241;
    size_t height = 376;

    // calibration parameters for odometery sequence 00
    libviso2::VisualOdometryStereo::parameters param;
    param.calib.f  = 718.85; // focal length in pixels
    param.calib.cu = 607.19; // principal point (u-coordinate) in pixels
    param.calib.cv = 185.21; // principal point (v-coordinate) in pixels
    param.base     = 0.5371; // baseline in meters

    //setup the camera calibration
    double fx, fy, s, px, py, b;
    fx = param.calib.f;
    fy = param.calib.f;
    s = 0;
    px = param.calib.cu;
    py = param.calib.cv;
    b = param.base;

    // construct the stereo calibration shared pointer, no need to delete it
    const gtsam::Cal3_S2Stereo::shared_ptr K(new gtsam::Cal3_S2Stereo(fx, fy, s, px, py, b));

    double sigmaPixel = 2;
    const gtsam::noiseModel::Isotropic::shared_ptr model = gtsam::noiseModel::Isotropic::Sigma(3, sigmaPixel);

    // init visual odometry
    auto *viso = new libviso2::VisualOdometryStereo(param);

    std::vector<int32_t> inliers;
    // Vector of structs p_match for storing matches.
    std::vector<libviso2::Matcher::p_match> pM;

    //// For display the pose, begin the thread
    // init the drawer
    SFO::Drawer poseDrawer(gtPosesFileName);
    // Start the drawer thread
    boost::thread threadDrawer(boost::bind(&SFO::Drawer::start, poseDrawer));
/////////////////////////////////////////////

    // current pose (this matrix transforms a point from the current
    // frame's camera coordinates to the first frame's camera coordinates)
    libviso2::Matrix gtsamPose = libviso2::Matrix::eye(4);
    libviso2::Matrix libviso2Pose = libviso2::Matrix::eye(4);

    ////// the third pointer
    auto *gtsamPoseVec = new std::vector<libviso2::Matrix>();
    auto *libviso2PoseVec = new std::vector<libviso2::Matrix>();

    gtsamPoseVec->push_back(gtsamPose);
    libviso2PoseVec->push_back(libviso2Pose);

    poseDrawer.updatePoses(gtsamPoseVec);
    poseDrawer.updatePoseslib(libviso2PoseVec);

    char frameFileName[256];
    cv::String leftImgPath, rightImgPath;
    png::image<png::gray_pixel> leftImg, rightImg;
    auto *left_img_data = (uint8_t *)malloc(width * height * sizeof(uint8_t));
    auto *right_img_data = (uint8_t *)malloc(width * height * sizeof(uint8_t));

    // compute visual odometry
    std::int32_t dims[] = {static_cast<std::int32_t>(width),
                           static_cast<std::int32_t>(height),
                           static_cast<std::int32_t>(width)};

    for (std::size_t i = 0; i < 4541; i++) {

        // input file name
        sprintf(frameFileName, "%06zu.png", i);
        leftImgPath  = sequenceDirPath + "/image_0/" + frameFileName;
        rightImgPath = sequenceDirPath + "/image_1/" + frameFileName;

        try {
            // load left and right input image
            leftImg.read(leftImgPath);
            rightImg.read(rightImgPath);

            std::size_t v, u;
            std::size_t k = 0;
            for (v = 0; v < height; v++) {
                for (u = 0; u < width; u++) {
                    left_img_data[k]  = leftImg.get_pixel(u, v);
                    right_img_data[k] = rightImg.get_pixel(u, v);
                    k++;
                }
            }

            std::cout << "Processing: Frame: " << std::setw(4) << i;

            if (viso->process(left_img_data, right_img_data, dims)) {
                // on success, update current pose
                // get the matches
                pM.clear();
                inliers.clear();
                pM = viso->getMatches();
                inliers = viso->getInlierIndices();

                // initial values choices:
                //libviso2::Matrix poseInit = libviso2::Matrix::inv(viso->getMotion());
                libviso2::Matrix poseInit = libviso2::Matrix::eye(4);

                libviso2::Matrix poseOpt(4, 4);

                //TODO: Local Optimization to get an optimized relative pose and feature point
                SFO::localOptimization(pM, inliers, poseInit, param, sigmaPixel, K, model, poseOpt);

                gtsamPose = gtsamPose * (poseOpt);
                gtsamPoseVec->push_back(gtsamPose);

                libviso2Pose = libviso2Pose * libviso2::Matrix::inv(viso->getMotion());
                libviso2PoseVec->push_back(libviso2Pose);

                // output some statistics
                double num_matches = viso->getNumberOfMatches();
                double num_inliers = viso->getNumberOfInliers();
                std::cout << ", Matches: " << std::fixed << std::setprecision(0) << num_matches;
                std::cout << ", Inliers: " << std::fixed << std::setprecision(2)
                          << 100.0 * num_inliers / num_matches << " %" << std::endl;

                poseDrawer.updatePoses(gtsamPoseVec);
                poseDrawer.updatePoseslib(libviso2PoseVec);

                // Display nice image,
                //cv::Mat img_C1 = Mat(height, width, CV_8UC1, left_img_data);
                //cv::Mat img_C2 = Mat(height, width, CV_8UC1, right_img_data);
                cv::Mat img_C1 = cv::imread(leftImgPath, cv::IMREAD_GRAYSCALE);
                cv::Mat img_C2 = cv::imread(rightImgPath, cv::IMREAD_GRAYSCALE);
                cv::Size sz1 = img_C1.size();
                cv::Size sz2 = img_C2.size();

                // Create combined matrix
                cv::Mat im3(sz1.height + sz2.height + 5, sz1.width, CV_8UC1);
                cv::Mat left(im3, cv::Rect(0, 0, sz1.width, sz1.height));
                img_C1.copyTo(left);
                cv::Mat right(im3, cv::Rect(0, sz1.height + 5, sz2.width, sz2.height));
                img_C2.copyTo(right);

                // Convert to color type
                cv::cvtColor(im3, im3, CV_GRAY2RGB);

                // ==============================
                // Add extracted features to mat
                // ==============================
                for (std::size_t ki = 0; ki < pM.size(); ki++) {
                    // Get match data
                    libviso2::Matcher::p_match match;
                    match = pM.at(ki);
                    // Point color object
                    cv::Scalar color;
                    // Check to see if inlier, if so color it as one
                    // The outliers are red
                    if (std::find(inliers.begin(), inliers.end(), ki) != inliers.end()) {
                        color = CV_RGB(0, 255, 255);
                    } else {
                        color = CV_RGB(255, 0, 0);
                    }
                    // Add to image
                    cv::Point2f pt_left(match.u1c, match.v1c);
                    cv::circle(im3, pt_left, 2, color);
                    cv::Point2f pt_right(match.u2c, match.v2c + (sz1.height + 5));
                    cv::circle(im3, pt_right, 2, color);
                }

                // Display it
                cv::imshow("Stereo Gray Image", im3);
                cv::waitKey(10);

                // Free the data once done
                im3.release();

                // Done delete them
                // Memory will grow unbounded otherwise
                img_C1.release();
                img_C2.release();

            } else if(i == 0) {
                std::cout << std::endl;
            } else {
                std::cerr << " ... failed!" << std::endl;
            }


            // catch image read errors here
        } catch (...) {
            std::cerr << "ERROR: Couldn't read input files!" << std::endl;
            return 1;
        }
    } //end for(int32_t i = 0; i < 4500; i++)

    // release uint8_t buffers
    free(left_img_data);
    free(right_img_data);


    // Stop the visualization thread if it was running
    // https://www.quantnet.com/threads/c-multithreading-in-boost.10028/
    while (!threadDrawer.timed_join(boost::posix_time::seconds(1))) {
        threadDrawer.interrupt();
    }

    std::cout << "SFO_main complete! Exiting ..." << std::endl;


	return 0;
}
