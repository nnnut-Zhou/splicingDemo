//
// Created by root on 2026/1/7.
//
#include <algorithm>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <openMVG/sfm/sfm_data.hpp>
#include <openMVG/sfm/sfm_data_io.hpp>
#include <openMVG/sfm/sfm_data_utils.hpp>
#include <openMVG/image/image_io.hpp>
#include <openMVG/exif/exif_IO_EasyExif.hpp>
#include <openMVG/cameras/cameras_io.hpp>

#include "DadeInterface.h"

namespace fs = boost::filesystem;
namespace sfm = openMVG::sfm;

namespace {
    auto& db = dade::DadeCameraDB::getInstance();

    double get_pix_focal(const double real_focal, const double pix_width,
                         const double real_width) {
        return real_focal * pix_width / real_width;
    }

}

namespace dade {

    DadeErr DadeList::ExtractList(
        bool group_camera_model,
        EINTRINSIC camera_model_type,
        COORDI_LIST_TYPE coordi_list_type,
        const std::string& sfm_out
    ) {
        // check
        const fs::path image_dir_p(image_dir_);
        if (!fs::exists(image_dir_p) || !fs::is_directory(image_dir_p)) {
            std::cerr << "The input image_dir doesn't exist" << std::endl;
            return 1;
        }
        const fs::path sfm_out_p(sfm_out);
        if (sfm_out_p.extension() != ".json") {
            std::cerr << "The format of output sfm_out is invalid" << std::endl;
            return 1;
        }
        const fs::path sfm_out_dir_p = sfm_out_p.parent_path();
        if (!sfm_out_dir_p.empty() && !fs::exists(sfm_out_dir_p)) {
            if (!fs::create_directories(sfm_out_dir_p)) {
                std::cerr << "Cannot create output sfm_out's directory: " << sfm_out_dir_p << std::endl;
                return 1;
            }
        }

        // 读取照片
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

        sfm_data_.s_root_path = image_dir_;
        sfm::Views& sfm_views = sfm_data_.views;
        sfm::Intrinsics& sfm_intrinsics = sfm_data_.intrinsics;

        // check
        if (pose_.GetPosPair().size() != vec_image.size()) { return 1; }

        // pos转至目标坐标系
        bool have_no_pose = false;
        double center_X, center_Y, center_Z;
        switch (coordi_list_type) {
            case COORDI_LIST_TYPE::COORDINATE_XYZ:
                pose_.TransToXYZ(center_X, center_Y, center_Z);
                break;
            case COORDI_LIST_TYPE::COORDINATE_UTM:
                pose_.TransToUTM(center_X, center_Y, center_Z);
                break;
            case COORDI_LIST_TYPE::COORDINATE_LOCAL:
                pose_.TransToLocalUTM(center_X, center_Y, center_Z);
                break;
            default:
                have_no_pose = true;
        }

        bool cParam_is_defined = calib_params_.IsDefined();

        // 循环读图，构造内参对象Intrinsic，更新sfm容器
        for (size_t i = 0; i < vec_image.size(); ++i) {
            double width, height, ppx, ppy, focal;

            const std::string& image_name = vec_image[i];
            const std::string full_path = (image_dir_p / image_name).string();
            openMVG::image::ImageHeader img_header{};
            if (!openMVG::image::ReadImageHeader(full_path.c_str(), &img_header)) {
                std::cerr << "Cannot read image header: " << image_name;
                continue;
            }
            width = img_header.width;
            height = img_header.height;
            ppx = width / 2.0;
            ppy = height / 2.0;

            if (cParam_is_defined) {
                std::cout << "cParam is defined" << std::endl;
                ppx = calib_params_.principal_point_x;
                ppy = calib_params_.principal_point_y;
                focal = std::max(calib_params_.focal_length_x, calib_params_.focal_length_y);
            } else {
                auto exif_reader = std::make_unique<openMVG::exif::Exif_IO_EasyExif>();
                if (exif_reader->open(full_path)) {
                    auto focal_mm = exif_reader->getFocal();
                    if (focal_mm > 0) {
                        // 根据相机参数计算像素焦距
                        std::string camera_brand = exif_reader->getBrand();
                        std::string camera_model = exif_reader->getModel();
                        const std::string s_model = camera_brand.append(" ").append(camera_model);
                        double real_width;
                        if (db.getSensorWidth(s_model, real_width)) {
                            focal = get_pix_focal(focal_mm, width, real_width);
                        } else {
                            focal = std::max(width, height);
                        }
                    } else {
                        focal = std::max(width, height);
                    }
                } else {
                    focal = std::max(width, height);
                }
            }

            // 根据选择的相机模型，构造不同的Intrinsic
            std::shared_ptr<openMVG::cameras::IntrinsicBase> intrinsic_ptr = nullptr;
            if (focal > 0) {
                switch (camera_model_type) {
                    case EINTRINSIC::PINHOLE_CAMERA:
                        intrinsic_ptr = std::make_shared<openMVG::cameras::Pinhole_Intrinsic>(
                            width, height, focal, ppx, ppy);
                        break;
                    case EINTRINSIC::PINHOLE_CAMERA_RADIAL1:
                        intrinsic_ptr = std::make_shared<openMVG::cameras::Pinhole_Intrinsic_Radial_K1>(
                            width, height, focal, ppx, ppy, 0.0);
                        break;
                    case EINTRINSIC::PINHOLE_CAMERA_RADIAL3:
                        intrinsic_ptr = std::make_shared<openMVG::cameras::Pinhole_Intrinsic_Radial_K3>(
                            width, height, focal, ppx, ppy, 0.0, 0.0, 0.0);
                        break;
                    case EINTRINSIC::PINHOLE_CAMERA_BROWN:
                        intrinsic_ptr = std::make_shared<openMVG::cameras::Pinhole_Intrinsic_Brown_T2>(
                            width, height, focal, ppx, ppy, 0.0, 0.0, 0.0, 0.0, 0.0);
                        break;
                    default:
                        std::cerr << "Unknown camera model type";
                        return 1;
                }
            }

            const auto id_view = static_cast<uint32_t>(i);
            const uint32_t id_intrinsic = (intrinsic_ptr != nullptr) ? id_view : openMVG::UndefinedIndexT;

            // 构造View，并绑定id放入sfm容器
            if (have_no_pose) {
                auto v = std::make_shared<sfm::View>(
                    image_name, id_view, id_intrinsic, id_view, width, height);
                sfm_views[id_view] = v;
            } else {
                // ViewPriors继承View，多了先验位姿信息（pos）
                auto v_prior = std::make_shared<sfm::ViewPriors>(
                    image_name, id_view, id_intrinsic, id_view, width, height);
                auto it_pos = pose_.GetPosPair().find(static_cast<int>(i));
                if (it_pos != pose_.GetPosPair().end()) {
                    // gps先验标记为可用
                    v_prior->b_use_pose_center_ = true;
                    // 另一个b_use_pose_rotation_是旋转姿态先验标记
                    // 测试集的云台角度误差较大，故不使用
                    v_prior->pose_center_ = openMVG::Vec3(
                        it_pos->second.longitude, it_pos->second.latitude, it_pos->second.altitude);
                }
                sfm_views[id_view] = v_prior;
            }

            if (intrinsic_ptr) {
                sfm_intrinsics[id_intrinsic] = intrinsic_ptr;
            }
        }

        // 相机内参分组合并，同一台相机的多个内参会合并成一个，加速并提高稳定性
        if (group_camera_model) {
            openMVG::sfm::GroupSharedIntrinsics(sfm_data_);
        }

        // 写入到文件
        // flags_part可指定保存sfm容器的哪些信息
        if (!Save(
            sfm_data_,
            sfm_out,
            static_cast<sfm::ESfM_Data>(
                sfm::ESfM_Data::VIEWS | sfm::ESfM_Data::INTRINSICS))) {
            return 1;
        }

        return 0;
    }

}
