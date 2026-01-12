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
    class Dade {
    public:
        virtual ~Dade() = default;

        static void SoftwareInfo() {
            std::cout << "Software Info" << std::endl
                    << "Developer: LeeChou" << std::endl
                    << "Version: 0.1" << std::endl;
        }
    };

    class DadeGeometry : public Dade {
    public:
        // WGS84: ECEF to lat-lon
        static openMVG::Vec3 XYZToLatLon(double x, double y, double z);

        static openMVG::Vec3 LatLonToXYZ(double lat, double lon, double alt);

        // UTM Mercator
        static openMVG::Vec3 LatLonToUTM(double lat, double lon, double alt);

        // Web Mercator
        static openMVG::Vec3 WMTToLatLon(double x, double y, double z);

        static openMVG::Vec3 LatLonToWMT(double lat, double lon, double alt);

        // GCJ02
        static bool OutOfChina(double lat, double lon);

        static void LatLonToGCJ(double wgLon, double wgLat, double& mgLon, double& mgLat);
    };

    class DadePose : public Dade {
    public:
        POSPair pos_pair_;

        virtual DadeErr GetPOS(const std::string& image_dir) = 0;

        virtual DadeErr GetPOSFromPOSFile(const std::string& pose_file) = 0;

        virtual DadeErr ExportPOS(const std::string& output_file) = 0;

        // return center of UTM && translate all pos to UTM
        DadeErr ExtractUTM(double& center_x, double& center_y, double& center_z);

        // return center of ECEF && translate all pos to ECEF
        DadeErr ExtractXYZ(double& center_x, double& center_y, double& center_z);

        // return center of UTM
        // translate all pos from latlon to UTM, and consider the center as the origin, except z
        DadeErr ExtractLocalUTM(double& center_x, double& center_y, double& center_z);
    };


    class DadeList : public Dade {
    public:
        static DadeErr GetList(
            const std::string& image_dir,
            const std::string& pos_file,
            CalibParams& cParam,
            bool group_camera_model,
            EINTRINSIC camera_model_type,
            COORDI_LIST_TYPE coordi_list_type,
            DadePose& dade_pose,
            const std::string& sfm_out
        );

        // virtual DadeErr UAVProcessListRange(CalibParams cParam, POSPair posList) {
        //     return 0;
        // }
        //
        // virtual double UAVProcessListSize(std::string dImage) {
        //     return 0;
        // };
    };
}

#endif //DADESPLICING_DADEINTERFACE_H
