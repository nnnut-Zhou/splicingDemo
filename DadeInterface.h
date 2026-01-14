//
// Created by root on 2026/1/2.
//

#ifndef DADESPLICING_DADEINTERFACE_H
#define DADESPLICING_DADEINTERFACE_H
#include <iostream>
#include <map>
#include <ostream>
#include <utility>
#include <openMVG/numeric/eigen_alias_definition.hpp>
#include <openMVG/sfm/sfm_data.hpp>

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
        ~DadeGeometry() override = default;

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

    // 单例
    class DadeCameraDB : public Dade {
    private:
        bool is_loaded_ = false;

        std::map<std::string, double> database_{};

        // 防止外界创建
        DadeCameraDB() = default;

        // 防止外界删除
        ~DadeCameraDB() override = default;

    public:
        static DadeCameraDB& getInstance() {
            static DadeCameraDB instance;
            return instance;
        }

        DadeCameraDB(const DadeCameraDB&) = delete;

        DadeCameraDB& operator=(const DadeCameraDB&) = delete;

        bool load(const std::string& dbPath);

        bool getSensorWidth(const std::string& model, double& width) const;
    };

    class DadePose : public Dade {
    private:
        POSPair pos_pair_{};

    public:
        ~DadePose() override = default;

        // 显式提供move
        DadePose(DadePose&&) = default;

        DadePose& operator=(DadePose&&) = default;

        // 禁止拷贝
        DadePose(const DadePose&) = delete;

        DadePose& operator=(const DadePose&) = delete;

        DadePose() = default;

        explicit DadePose(const std::string& image_dir) {
            ExtractPOS(image_dir);
        }

        const POSPair& GetPosPair() {
            return pos_pair_;
        }

        DadeErr ExtractPOS(const std::string& image_dir);

        // TODO not implemented yet
        DadeErr ExtractPOSFromPOSFile(const std::string& pose_file);

        // TODO not implemented yet
        DadeErr ExportPOS(const std::string& output_file);

        // return center of UTM && translate all pos to UTM
        DadeErr TransToUTM(double& center_x, double& center_y, double& center_z);

        // return center of ECEF && translate all pos to ECEF
        DadeErr TransToXYZ(double& center_x, double& center_y, double& center_z);

        // return center of UTM
        // translate all pos from latlon to UTM, and consider the center as the origin, except z
        DadeErr TransToLocalUTM(double& center_x, double& center_y, double& center_z);
    };


    class DadeList : public Dade {
    private:
        DadePose pose_{};

        CalibParams calib_params_{};

        std::string image_dir_{};

        // openMVG sfm容器
        openMVG::sfm::SfM_Data sfm_data_{};

    public:
        ~DadeList() override = default;

        // 显式提供move
        DadeList(DadeList&&) = default;

        DadeList& operator=(DadeList&&) = default;

        // 禁止拷贝
        DadeList(const DadeList&) = delete;

        DadeList& operator=(const DadeList&) = delete;

        DadeList() = default;

        DadeList(
            DadePose pose,
            const CalibParams& cParam,
            std::string image_dir
        ) : pose_(std::move(pose)), calib_params_(cParam), image_dir_(std::move(image_dir)) {
        }

        DadeErr ExtractList(
            bool group_camera_model,
            EINTRINSIC camera_model_type,
            COORDI_LIST_TYPE coordi_list_type,
            const std::string& sfm_out
        );

        // TODO extract list from file
        DadeErr ExtractList() { return 1; }

    };

    class DadeFeature : public Dade {
    private:
        std::map<int, FeatureParams> features_{};

        DadeList dade_list_{};

        // TODO ====== 由已有数据填features_ ======
        void init();

    public:
        ~DadeFeature() override = default;

        // 显式提供move
        DadeFeature(DadeFeature&&) = default;

        DadeFeature& operator=(DadeFeature&&) = default;

        // 禁止拷贝
        DadeFeature(const DadeFeature&) = delete;

        DadeFeature& operator=(const DadeFeature&) = delete;

        DadeFeature() = default;

        explicit DadeFeature(DadeList dade_list) : dade_list_(std::move(dade_list)) {
            init();
        }

        // TODO init from file
        explicit DadeFeature(std::string& sfm_path) {
            init();
        }


    };


}

#endif //DADESPLICING_DADEINTERFACE_H
