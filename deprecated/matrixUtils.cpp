#include "matrixUtils.h"

#include <gtsam/geometry/Point3.h>

namespace SFO {
    void Tx(libviso2::Matrix &T, double angle) {
        T = libviso2::Matrix(4, 4);
        T.setMat(libviso2::Matrix::rotMatX(angle), 0, 0);
        T.val[3][3] = 1;
    }

    void Ty(libviso2::Matrix &T, double angle) {
        T = libviso2::Matrix(4, 4);
        T.setMat(libviso2::Matrix::rotMatY(angle), 0, 0);
        T.val[3][3] = 1;
    }

    void Tz(libviso2::Matrix &T, double angle) {
        T = libviso2::Matrix(4, 4);
        T.setMat(libviso2::Matrix::rotMatZ(angle), 0, 0);
        T.val[3][3] = 1;
    }

    void Tx(gtsam::Pose3 &T, double angle) {
        T = gtsam::Pose3(gtsam::Rot3::Rx(angle), gtsam::Point3(0, 0, 0));
    }

    void Ty(gtsam::Pose3 &T, double angle) {
        T = gtsam::Pose3(gtsam::Rot3::Ry(angle), gtsam::Point3(0, 0, 0));
    }

    void Tz(gtsam::Pose3 &T, double angle) {
        T = gtsam::Pose(gtsam::Rot3::Rz(angle), gtsam::Point3(0, 0, 0));
    }

    void Rx(libviso2::Matrix &R, double angle){
        R = libviso2::Matrix::rotMatX(angle);
    }

    void Ry(libviso2::Matrix &R, double angle){
        R = libviso2::Matrix::rotMatY(angle);
    }

    void Rz(libviso2::Matrix &R, double angle){
        R = libviso2::Matrix::rotMatZ(angle);
    }

    void Rx(gtsam::Rot3 &R, double angle){
        R = gtsam::Rot3::Rx(angle);
    }

    void Ry(gtsam::Rot3 &R, double angle){
        R = gtsam::Rot3::Ry(angle);
    }

    void Rz(gtsam::Rot3 &R, double angle){
        R = gtsam::Rot3::Rz(angle);
    }

} // namespace SFO