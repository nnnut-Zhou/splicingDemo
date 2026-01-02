//
// Created by root on 2026/1/2.
//

#ifndef DADESPLICING_DADEINTERFACE_H
#define DADESPLICING_DADEINTERFACE_H
#include <iostream>
#include <map>
#include <ostream>

#include "common.h"

namespace dade {
    using POSPair = std::map<int, POS>;

    class Dade {
    public:
        virtual ~Dade() = default;

        static void SoftwareInfo() {
            std::cout << "Software Info" << std::endl
                    << "Developer: LeeChou" << std::endl
                    << "Version: 0.1" << std::endl;
        }
    };

    class DadePose : public Dade {
    public:
        POSPair pos_pair_;

        virtual DadeErr DadePOSGet(const std::string& image_dir, bool has_pose_file) = 0;
    };
}

#endif //DADESPLICING_DADEINTERFACE_H
