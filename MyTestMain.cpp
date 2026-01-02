#include <iostream>
#include <openMVG/numeric/eigen_alias_definition.hpp>
#include <boost/filesystem.hpp>

#include "DadePreprocess.h"

int main() {
    const std::string image_dir("/home/projects/DadeSplicing/resource/images");

    dade::preprocess::DadePOSExtractor extractor;
    dade::DadeErr err = extractor.DadePOSGet(image_dir, false);
    std::cout << err << std::endl;


    return 0;
}
