#include <iostream>
#include <openMVG/numeric/eigen_alias_definition.hpp>
#include <boost/filesystem.hpp>

#include "DadePose.h"
#include "DadePreprocess.h"

void DadeListTest() {
    const std::string image_dir("/home/projects/DadeSplicing/resource/images");
    dade::CalibParams cParam{};
    dade::DadePOSExtractor dade_pose{};
    const std::string file_out("/home/projects/DadeSplicing/resource/sfm_data.json");

    dade::DadeList::GetList(
        image_dir,
        "",
        cParam,
        true,
        dade::EINTRINSIC::PINHOLE_CAMERA,
        dade::COORDI_LIST_TYPE::COORDINATE_UTM,
        dade_pose, file_out
    );
}

int main() {
    DadeListTest();


    return 0;
}
