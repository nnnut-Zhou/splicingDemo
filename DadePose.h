//
// Created by root on 2026/1/7.
//

#ifndef DADESPLICING_DADEPOSE_H
#define DADESPLICING_DADEPOSE_H


#include <numeric/eigen_alias_definition.hpp>

#include "common.h"
#include "DadeInterface.h"

namespace dade {

    // 测试用pos提取器，可作为由pos直接投影的入口
    class DadePOSExtractorTest : public DadePose {
    public:
        ~DadePOSExtractorTest() override = default;

        DadeErr ExtractPOSToP(const CalibParams& intrinsic, std::vector<openMVG::Mat34>& vec_P);

        DadeErr ComputeP(int idx, const CalibParams& intrinsic, const openMVG::Vec3& placement_angle,
                         const openMVG::Vec3& placement_vec, openMVG::Mat34& P);
    };

}


#endif //DADESPLICING_DADEPOSE_H
