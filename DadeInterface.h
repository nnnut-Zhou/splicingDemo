//
// Created by root on 2026/1/2.
//

#ifndef DADESPLICING_DADEINTERFACE_H
#define DADESPLICING_DADEINTERFACE_H
#include <iostream>
#include <map>
#include <ostream>

#include "common.h"

namespace dade {
    using POSPair = std::map<int, POS>;

    class Dade {
    public:
        virtual ~Dade() = default;

        static void SoftwareInfo() {
            std::cout << "Software Info" << std::endl
                    << "Developer: LeeChou" << std::endl
                    << "Version: 0.1" << std::endl;
        }
    };

    class DadePose : public Dade {
    public:
        POSPair pos_pair_;

        virtual DadeErr GetPOS(const std::string& image_dir, bool has_pose_file) = 0;
    };

    class DadeGeometry : public Dade {
    public:
        static openMVG::Vec3 XYZToLatLon(double x, double y, double z);

        static openMVG::Vec3 LatLonToXYZ(double lat, double lon, double alt);

        static openMVG::Vec3 LatLonToUTM(double lat, double lon, double alt);

        //web Mector
        static openMVG::Vec3 WMTToLatLon(double x, double y, double z);

        static openMVG::Vec3 LatLonToWMT(double lat, double lon, double alt);

        //GCJ02
        static bool OutOfChina(double lat, double lon);

        static void LatLonToGCJ(double wgLon, double wgLat, double& mgLon, double& mgLat);
    };
}

#endif //DADESPLICING_DADEINTERFACE_H
