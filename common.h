//
// Created by root on 2026/1/2.
//

#ifndef DADESPLICING_COMMON_H
#define DADESPLICING_COMMON_H

namespace dade {
    using DadeErr = int;

    struct POS {
        double latitude = 0.0; // dB，纬度，y
        double longitude = 0.0; // dL，经度，x
        double altitude = 0.0; // dH，高度，z
        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;
    };

    using POSPair = std::map<int, POS>;

    struct CalibParams {
        double focal_length_x = 0.0;
        double focal_length_y = 0.0;
        double principal_point_x = 0.0;
        double principal_point_y = 0.0;
        double ccd_size = 0.0;

        // set all attrs to default value (0.0)
        void Reset() {
            *this = {};
        }

        bool IsDefined() const {
            return focal_length_x != 0.0 ||
                   focal_length_y != 0.0 ||
                   principal_point_x != 0.0 ||
                   principal_point_y != 0.0 ||
                   ccd_size != 0.0;
        }
    };

    // camera model
    // copied from openMVG, with fisheye and spherical removed
    enum class EINTRINSIC : int {
        PINHOLE_CAMERA_START = 0,
        PINHOLE_CAMERA = 1, // No distortion
        PINHOLE_CAMERA_RADIAL1 = 2, // radial distortion K1
        PINHOLE_CAMERA_RADIAL3 = 3, // radial distortion K1,K2,K3
        PINHOLE_CAMERA_BROWN = 4, // radial distortion K1,K2,K3, tangential distortion T1,T2
        PINHOLE_CAMERA_END = 5
    };

    enum class COORDI_LIST_TYPE : int {
        COORDINATE_UTM = 0,
        COORDINATE_XYZ = 1,
        COORDINATE_LOCAL = 2
    };
}

#endif //DADESPLICING_COMMON_H
