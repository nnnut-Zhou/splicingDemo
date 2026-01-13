//
// Created by root on 2026/1/13.
//
#include <fstream>
#include <sstream>

#include "DadeInterface.h"

namespace {
    std::string trim(const std::string& s) {
        const auto start = s.find_first_not_of(" \t\r\n");
        if (start == std::string::npos) return "";
        const auto end = s.find_last_not_of(" \t\r\n");
        return s.substr(start, end - start + 1);
    }
}

namespace dade {
    bool DadeCameraDB::load(const std::string& dbPath) {
        if (is_loaded_) return true;

        std::ifstream file(dbPath);
        if (!file.is_open()) return false;

        std::string line;
        while (std::getline(file, line)) {
            std::string processedLine = trim(line);
            if (processedLine.empty() || processedLine[0] == '#') {
                continue;
            }

            std::stringstream ss(processedLine);
            std::string s_model, widthStr;

            // 格式为: s_model;width
            if (std::getline(ss, s_model, ';') &&
                std::getline(ss, widthStr, ';')) {
                database_[s_model] = std::stod(widthStr);
            }
        }
        is_loaded_ = true;
        return true;
    }

    bool DadeCameraDB::getSensorWidth(const std::string& model, double& width) const {
        const auto it = database_.find(model);
        if (it != database_.end()) {
            width = it->second;
            return true;
        }
        return false;
    }

}
