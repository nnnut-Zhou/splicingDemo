//
// Created by root on 2026/1/7.
//
#include <boost/filesystem.hpp>
#include <openMVG/exif/exif_IO_EasyExif.hpp>

#include "DadePose.h"
#include "common.h"

namespace fs = boost::filesystem;

namespace dade {
    DadeErr DadePOSExtractor::GetPOS(const std::string& image_dir) {
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
                    pos_pair_.emplace(image_index++, POS{lat, lon, alt, 0.0, 0.0, 0.0});
                    ++image_index;
                } else {
                    // gps something lack
                    return 1;
                }
            } else {
                // can't opem or exif lack
                return 1;
            }
        }
        return 0;
    }

    DadeErr DadePose::ExtractUTM(double& center_x, double& center_y, double& center_z) {
        const size_t size = pos_pair_.size();
        if (size == 0)
            return 1;

        center_x = 0;
        center_y = 0;
        center_z = 0;
        for (auto& iter: pos_pair_) {
            openMVG::Vec3 utm = DadeGeometry::LatLonToUTM(
                iter.second.latitude, iter.second.longitude, iter.second.altitude
            );
            iter.second.latitude = utm(0);
            iter.second.longitude = utm(1);
            iter.second.altitude = utm(2);
            center_x += utm(0);
            center_y += utm(1);
            center_z += utm(2);
        }
        const auto d_size = static_cast<double>(size);
        center_x /= d_size;
        center_y /= d_size;
        center_z /= d_size;

        return 0;
    }

    DadeErr DadePose::ExtractXYZ(double& center_x, double& center_y, double& center_z) {
        const size_t size = pos_pair_.size();
        if (size == 0)
            return 1;

        center_x = 0;
        center_y = 0;
        center_z = 0;
        for (auto& iter: pos_pair_) {
            openMVG::Vec3 xyz = DadeGeometry::LatLonToXYZ(
                iter.second.latitude, iter.second.longitude, iter.second.altitude
            );
            iter.second.latitude = xyz(0);
            iter.second.longitude = xyz(1);
            iter.second.altitude = xyz(2);
            center_x += xyz(0);
            center_y += xyz(1);
            center_z += xyz(2);
        }
        const auto d_size = static_cast<double>(size);
        center_x /= d_size;
        center_y /= d_size;
        center_z /= d_size;

        return 0;
    }

    DadeErr DadePose::ExtractLocalUTM(double& center_x, double& center_y, double& center_z) {
        const size_t size = pos_pair_.size();
        if (size == 0)
            return 1;

        center_x = 0;
        center_y = 0;
        center_z = 0;
        for (auto& iter: pos_pair_) {
            openMVG::Vec3 utm = DadeGeometry::LatLonToUTM(
                iter.second.latitude, iter.second.longitude, iter.second.altitude
            );
            iter.second.latitude = utm(0);
            iter.second.longitude = utm(1);
            iter.second.altitude = utm(2);
            center_x += utm(0);
            center_y += utm(1);
            center_z += utm(2);
        }
        const auto d_size = static_cast<double>(size);
        center_x /= d_size;
        center_y /= d_size;
        center_z /= d_size;

        for (auto& iter : pos_pair_) {
            iter.second.latitude -= center_x;
            iter.second.longitude -= center_y;
            // iter.second.altitude -= center_z;
        }

        return 0;
    }


}
