//
// Created by root on 2026/1/2.
//

#ifndef DADESPLICING_COMMON_H
#define DADESPLICING_COMMON_H

namespace dade {
    using DadeErr = int;

    struct POS {
        double longitude = 0.0;
        double latitude = 0.0;
        double altitude = 0.0;
        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;
    };

    struct CalibParams {
        double focal_length_x = 0.0;
        double focal_length_y = 0.0;
        double principal_point_x = 0.0;
        double principal_point_y = 0.0;
        double ccd_size = 0.0;
    };
}

#endif //DADESPLICING_COMMON_H
