//
// Created by root on 2026/1/2.
//

#include "DadePreprocess.h"
#include <boost/filesystem.hpp>
#include <openMVG/exif/exif_IO_EasyExif.hpp>

namespace fs = boost::filesystem;

namespace dade {
    namespace preprocess {
        DadeErr DadePOSExtractor::DadePOSGet(const std::string& image_dir, const bool has_pose_file) {
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

        long DadePOSExtractor::ExtractPOSToP(std::vector<openMVG::Mat34>& vec_P, CalibParams instric) {
            return 0;
        }
    }
}
