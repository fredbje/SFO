#ifndef SFO_MATRIXUTILS_H
#define SFO_MATRIXUTILS_H

#include "libviso2::Matrix.h"
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>

namespace SFO {
    // Transformation matrix with rotation around the x, y, or z axis
    void Tx(libviso2::Matrix &T, double angle);
    void Ty(libviso2::Matrix &T, double angle);
    void Tz(libviso2::Matrix &T, double angle);

    void Tx(gtsam::Pose3 &T, double angle);
    void Ty(gtsam::Pose3 &T, double angle);
    void Tz(gtsam::Pose3 &T, double angle);

    // Rotation matrix with rotation around the x, y, or z axis
    void Rx(libviso2::Matrix &R, double angle);
    void Ry(libviso2::Matrix &R, double angle);
    void Rz(libviso2::Matrix &R, double angle);

    void Rx(gtsam::Rot3 &R, double angle);
    void Ry(gtsam::Rot3 &R, double angle);
    void Rz(gtsam::Rot3 &R, double angle);

} // namespace SFO

#endif //SFO_MATRIXUTILS_H
