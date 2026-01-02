//
// Created by root on 2026/1/2.
//

#ifndef DADESPLICING_DADEPREPROCESS_H
#define DADESPLICING_DADEPREPROCESS_H

#include <vector>
#include <numeric/eigen_alias_definition.hpp>

#include "DadeInterface.h"


namespace dade {
    namespace preprocess {
        class DadePOSExtractor : public DadePose {
        public:
            DadeErr GetPOS(const std::string& image_dir, bool has_pose_file) override;

            // transfer POS to Projection Matrix
            DadeErr ExtractPOSToP(const CalibParams& instric, std::vector<openMVG::Mat34>& vec_P);

            DadeErr ComputeP(int idx, const CalibParams& instric, openMVG::Vec3f placement_angle,
                             openMVG::Vec3f placement_vec, openMVG::Mat34& P);
        };
    }
}

#endif //DADESPLICING_DADEPREPROCESS_H
