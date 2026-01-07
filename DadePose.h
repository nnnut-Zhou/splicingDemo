//
// Created by root on 2026/1/7.
//

#ifndef DADESPLICING_DADEPOSE_H
#define DADESPLICING_DADEPOSE_H


#include <numeric/eigen_alias_definition.hpp>

#include "common.h"
#include "DadeInterface.h"

namespace dade {
    class DadePOSExtractor : public DadePose {
    public:
        DadeErr GetPOS(const std::string& image_dir) override;

        // TODO not implemented yet
        DadeErr GetPOSFromPOSFile(const std::string& pose_file) override { return 0; };

        // TODO not implemented yet
        DadeErr ExportPOS(const std::string& output_file) override { return 0; };
    };

    // simple project to ground, just for test
    class DadePOSExtractorTest : public DadePOSExtractor {
    public:
        DadeErr ExtractPOSToP(const CalibParams& intrinsic, std::vector<openMVG::Mat34>& vec_P);

        DadeErr ComputeP(int idx, const CalibParams& intrinsic, const openMVG::Vec3& placement_angle,
                         const openMVG::Vec3& placement_vec, openMVG::Mat34& P);
    };

}


#endif //DADESPLICING_DADEPOSE_H
