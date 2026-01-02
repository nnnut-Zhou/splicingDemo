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
            DadeErr DadePOSGet(const std::string& image_dir, bool has_pose_file) override;

            long ExtractPOSToP(std::vector<openMVG::Mat34>& vec_P, CalibParams instric);
        };
    }
}

#endif //DADESPLICING_DADEPREPROCESS_H
