#include <iostream>
#include <boost/filesystem.hpp>

#include "DadePose.h"
#include "DadePreprocess.h"

void DadeCameraDBTest() {
    const auto& db = dade::DadeCameraDB::getInstance();
    double width;
    if (db.getSensorWidth("DJI M3TD", width)) {
        std::cout << width << std::endl;
    } else {
        std::cerr << "Failed to get sensor width!" << std::endl;
    }
}

void DadeListTest() {
    const std::string image_dir("/home/projects/DadeSplicing/resource/images");
    const std::string sfm_out("/home/projects/DadeSplicing/output/sfm_data.json");

    dade::DadePose dade_pose(image_dir);
    dade::CalibParams cParam{};

    dade::DadeList dade_list(std::move(dade_pose), cParam, image_dir);
    dade_list.ExtractList(
        true,
        dade::EINTRINSIC::PINHOLE_CAMERA_BROWN,
        dade::COORDI_LIST_TYPE::COORDINATE_UTM,
        sfm_out
    );




}

int main() {
    // init
    const std::string dbPath = "/home/projects/DadeSplicing/resource/sensor_width_camera_database.txt";
    auto& db = dade::DadeCameraDB::getInstance();
    if (!db.load(dbPath)) {
        std::cerr << "Failed to load camera database" << std::endl;
        return 1;
    }

    // DadeCameraDBTest();

    DadeListTest();


    return 0;
}
