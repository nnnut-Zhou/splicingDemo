//
// Created by root on 2026/1/7.
//
#include <algorithm>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <openMVG/sfm/sfm_data.hpp>

// #include <openMVG/sfm/sfm.hpp>

#include "DadeInterface.h"

namespace fs = boost::filesystem;
namespace sfm = openMVG::sfm;

namespace dade {

    DadeErr DadeList::GetList(
        const std::string& image_dir,
        const std::string& pos_file,
        CalibParams& cParam,
        const bool group_camera_model,
        const EINTRINSIC camera_model_type,
        const COORDI_LIST_TYPE coordi_list_type,
        DadePose& dade_pose,
        const std::string& sfm_out
    ) {
        const fs::path image_dir_p(image_dir);
        if (!fs::exists(image_dir_p) || !fs::is_directory(image_dir_p)) {
            std::cerr << "The input image_dir doesn't exist" << std::endl;
            return 1;
        }
        const fs::path sfm_out_p(sfm_out);
        if (sfm_out_p.extension() != ".json") {
            std::cerr << "The format of output sfm_out is invalid" << std::endl;
            return 1;
        }
        // check the parent path of sfm_out exists, if not create it
        const fs::path sfm_out_dir_p = sfm_out_p.parent_path();
        if (!sfm_out_dir_p.empty() && !fs::exists(sfm_out_dir_p)) {
            if (!fs::create_directories(sfm_out_dir_p)) {
                std::cerr << "Cannot create output sfm_out's directory: " << sfm_out_dir_p << std::endl;
                return 1;
            }
        }

        // read images
        std::vector<std::string> vec_image;
        for (const auto& entry: fs::directory_iterator(image_dir_p)) {
            if (fs::is_regular_file(entry)) {
                auto ext = entry.path().extension().string();
                boost::to_lower(ext);
                if (ext == ".jpeg") {
                    vec_image.push_back(entry.path().filename().string());
                }
            }
        }
        std::sort(vec_image.begin(), vec_image.end());

        // sfm data declare
        sfm::SfM_Data sfm_data;
        sfm_data.s_root_path = image_dir;
        sfm::Views& views = sfm_data.views;
        sfm::Intrinsics& intrinsics = sfm_data.intrinsics;

        // get pos
        bool have_no_pose = true;
        const fs::path pos_file_p(pos_file);
        if (fs::exists(pos_file_p) && fs::is_regular_file(pos_file_p)) {
            have_no_pose = dade_pose.GetPOSFromPOSFile(pos_file);
        } else {
            have_no_pose = dade_pose.GetPOS(image_dir);
        }

        // check pos's number
        if (!have_no_pose && dade_pose.pos_pair_.size() != vec_image.size()) { return 1; }

        // translate origin latlon to computable coordinate
        double center_X, center_Y, center_Z;
        if (!have_no_pose) {
            switch (coordi_list_type) {
                case COORDI_LIST_TYPE::COORDINATE_XYZ:
                    dade_pose.ExtractXYZ(center_X, center_Y, center_Z);
                    break;
                case COORDI_LIST_TYPE::COORDINATE_UTM:
                    dade_pose.ExtractUTM(center_X, center_Y, center_Z);
                    break;
                case COORDI_LIST_TYPE::COORDINATE_LOCAL:
                    dade_pose.ExtractLocalUTM(center_X, center_Y, center_Z);
                    break;
                default:
                    have_no_pose = true;
            }
        }

        bool bCalibParam = cParam.IsUndefined();

        double width, height, ppx, ppy, focal;

        POSPair::iterator pos_iter = dade_pose->posList.begin();
        for (std::vector<std::string>::const_iterator iter_image = vec_image.begin();
             iter_image != vec_image.end();
             ++iter_image, ++pos_iter) {
            ppx = ppy = width = height = focal = -1;

            const std::string sImageFilename = stlplus::create_filespec(image_dir, *iter_image);
            const std::string sImFilenamePart = stlplus::filename_part(sImageFilename);

            openMVG::image::ImageHeader imgHeader;
            if (!openMVG::image::ReadImageHeader(sImageFilename.c_str(), &imgHeader))
                continue;

            width = imgHeader.width;
            height = imgHeader.height;
            ppx = width / 2.0;
            ppy = height / 2.0;
            if (bCalibParam) {
                ppx = cParam._ppx_;
                ppy = cParam._ppy_;
                focal = std::max(cParam._flen_x_, cParam._flen_y_);
            } else {
                ppx = width / 2.0;
                ppy = height / 2.0;
                std::unique_ptr<openMVG::exif::Exif_IO> exifPosReader(new openMVG::exif::Exif_IO_EasyExif);
                if (exifPosReader->open(sImageFilename))
                    focal = std::max(width, height) * exifPosReader->getFocal() / exifPosReader->getFocal();
                else
                    focal = std::max(width, height);
            }

            std::unique_ptr<openMVG::exif::Exif_IO> exifReader(new openMVG::exif::Exif_IO_EasyExif);
            exifReader->open(sImageFilename);
            focal = std::max(width, height);

            std::shared_ptr<openMVG::cameras::IntrinsicBase> intrinsic(NULL);
            if (focal > 0 && ppx > 0 && ppy > 0 && height > 0 && width > 0) {
                //initial intrinsic
                //get from openMVG
                switch (camera_model_type) {
                    case PINHOLE_CAMERA:
                        intrinsic = std::make_shared<openMVG::cameras::Pinhole_Intrinsic>
                                (width, height, focal, ppx, ppy);
                        break;
                    case PINHOLE_CAMERA_RADIAL1:
                        intrinsic = std::make_shared<openMVG::cameras::Pinhole_Intrinsic_Radial_K1>
                                (width, height, focal, ppx, ppy, 0.0); // setup no distortion as initial guess
                        break;
                    case PINHOLE_CAMERA_RADIAL3:
                        intrinsic = std::make_shared<openMVG::cameras::Pinhole_Intrinsic_Radial_K3>
                                (width, height, focal, ppx, ppy, 0.0, 0.0, 0.0); // setup no distortion as initial guess
                        break;
                    case PINHOLE_CAMERA_BROWN:
                        intrinsic = std::make_shared<openMVG::cameras::Pinhole_Intrinsic_Brown_T2>
                                (width, height, focal, ppx, ppy, 0.0, 0.0, 0.0, 0.0, 0.0);
                        // setup no distortion as initial guess
                        break;
                    default:
                        std::cerr << "Error: unknown camera model: " << (int) camera_model_type << std::endl;
                        return 1;
                }
            }

            if (have_no_pose) {
                View v(*iter_image, views.size(), views.size(), views.size(), width, height);
                if (intrinsic == NULL)
                    v.id_intrinsic = openMVG::UndefinedIndexT;
                else
                    intrinsics[v.id_intrinsic] = intrinsic;
                // Add the view to the sfm_container
                views[v.id_view] = std::make_shared<View>(v);
            } else {
                ViewPriors v(*iter_image, views.size(), views.size(), views.size(), width, height);
                // Add intrinsic related to the image (if any)
                if (intrinsic == NULL)
                    v.id_intrinsic = openMVG::UndefinedIndexT;
                else
                    intrinsics[v.id_intrinsic] = intrinsic;

                // Add the view to the sfm_container
                views[v.id_view] = std::make_shared<View>(v);
                v.b_use_pose_center_ = true;
                double x = pos_iter->second.dL;
                double y = pos_iter->second.dB;
                double z = pos_iter->second.dH;
                v.pose_center_ = openMVG::Vec3(x, y, z);
                views[v.id_view] = std::make_shared<ViewPriors>(v);
            }

        }

        //group camera that share common properties
        //make BS more stable and faster
        if (group_camera_model) {
            openMVG::sfm::GroupSharedIntrinsics(sfm_data);
        }

        if (!Save(
            sfm_data,
            sfm_out.c_str(),
            ESfM_Data(VIEWS | INTRINSICS))) {
            return 3;
        }

        return 0;
    }


}
