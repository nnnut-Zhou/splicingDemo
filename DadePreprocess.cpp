//
// Created by root on 2026/1/2.
//

#include "DadePreprocess.h"
#include <boost/filesystem.hpp>
#include <numeric/numeric.h>
#include <openMVG/exif/exif_IO_EasyExif.hpp>

namespace fs = boost::filesystem;

namespace dade {
    namespace preprocess {
        DadeErr DadePOSExtractor::GetPOS(
            const std::string& image_dir,
            const bool has_pose_file) {
            // [WARN] image_dir should be full path

            if (!has_pose_file) {
                std::vector<std::string> vec_image;
                const fs::path p(image_dir);
                if (!fs::exists(p) || !fs::is_directory(p)) {
                    return 1;
                }
                for (const auto& entry: fs::directory_iterator(p)) {
                    if (fs::is_regular_file(entry)) {
                        auto ext = entry.path().extension().string();
                        if (ext == ".jpeg") {
                            vec_image.push_back(entry.path().filename().string());
                        }
                    }
                }
                std::sort(vec_image.begin(), vec_image.end());

                const auto exif_reader = std::make_unique<openMVG::exif::Exif_IO_EasyExif>();
                int image_index = 0;
                for (const auto& image_name: vec_image) {
                    const auto full_path = (p / image_name).string();
                    if (exif_reader->open(full_path) && exif_reader->doesHaveExifInfo()) {
                        double lat = 0.0, lon = 0.0, alt = 0.0;
                        if (exif_reader->GPSLatitude(&lat) &&
                            exif_reader->GPSLongitude(&lon) &&
                            exif_reader->GPSAltitude(&alt)) {
                            this->pos_pair_.emplace(image_index++, POS{lat, lon, alt, 0.0, 0.0, 0.0});
                            ++image_index;
                        } else {
                            // gps information gap
                            return 1;
                        }
                    } else {
                        // the file cannot be opened or lacks exif
                        return 1;
                    }
                }
                return 0;
            } else {
                // TODO temporarily ignore the case where there is a POS file.
                return 1;
            }
        }

        DadeErr DadePOSExtractor::ExtractPOSToP(
            const CalibParams& instric,
            std::vector<openMVG::Mat34>& vec_P) {

            for (const auto item: this->pos_pair_) {
                openMVG::Vec3f plA(0, 0, 0), plV(0, 0, 0);
                openMVG::Mat34 P;
                ComputeP(item.first, instric, plA, plV, P);
                vec_P.push_back(P);
            }
            return 0;
        }

        DadeErr DadePOSExtractor::ComputeP(
            const int idx,
            const CalibParams& instric,
            openMVG::Vec3f placement_angle,
            openMVG::Vec3f placement_vec,
            openMVG::Mat34& P) {

            const auto it = this->pos_pair_.find(idx);
            if (it == this->pos_pair_.end()) {
                return 1;
            }
            const POS& pos = it->second;

            double lat = openMVG::R2D(pos.latitude);
            double lon = openMVG::R2D(pos.longitude);
            double alt = pos.altitude;
            double roll = openMVG::R2D(pos.roll);
            double pitch = openMVG::R2D(pos.pitch);
            double yaw = openMVG::R2D(pos.yaw);

            using namespace Eigen;

            Matrix3d EM;
            EM << -sin(lon), cos(lon), 0,
                    -sin(lat) * cos(lon), -sin(lat) * sin(lon), cos(lat),
                    cos(lat) * cos(lon), cos(lat) * sin(lon), sin(lat);

            Matrix3d EG;
            EG << -sin(lat) * cos(lon), -sin(lon), -cos(lat) * cos(lon),
                    -sin(lat) * sin(lon), cos(lon), -cos(lat) * sin(lon),
                    cos(lat), 0, -sin(lat);

            Matrix3d GI;
            GI = AngleAxisd(yaw, Vector3d::UnitZ()) *
                 AngleAxisd(pitch, Vector3d::UnitY()) *
                 AngleAxisd(roll, Vector3d::UnitX());

            Matrix3d CI;
            CI = AngleAxisd(placement_angle(2), Vector3d::UnitZ()) *
                 AngleAxisd(placement_angle(1), Vector3d::UnitY()) *
                 AngleAxisd(placement_angle(0), Vector3d::UnitX());

            Matrix3d IC;
            IC << 0, -1, 0,
                    -1, 0, 0,
                    0, 0, -1;

            Matrix3d IMMatrix = EM * EG * GI * CI * IC;

            // openMVG::Vec3 cur_xyz = UAVProcessGeometry::UAVProcessGeoBLHToXYZ(lat, lon, alt);
            // openMVG::Vec3 origin_xyz = UAVProcessGeometry::UAVProcessGeoBLHToXYZ(lat, lon, 0); // 以地面高度为原点参考


            return 0;
        }
    }
}
