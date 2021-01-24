#pragma once
#include "ssdk/ssdk_lpr.h"
#include <string>
#include <sstream>
#include <map>
#include <functional>
#include <fstream>

namespace SSDK {
  //-------------------------------------------------------------------------
  template <typename T>
  bool setValue(T& value, const std::string& value) {
    try {
      std::stringstream ss(value);
      ss >> value;
    }
    catch (std::exception& error) {
      std::cerr << error.what() << std::endl;
      return false;
    }
    return true;
  }

  //-------------------------------------------------------------------------
  bool setThreads(SSDK::CarTrackerParams& params, const std::string& value) {
    return setValue(params.thread_count, value);
  }

  //-------------------------------------------------------------------------
  bool setPlateMaxWidth(SSDK::CarTrackerParams& params, const std::string& value) {
    return setValue(params.max_plate_width, value);
  }

  //-------------------------------------------------------------------------
  bool setPlateMinWidth(SSDK::CarTrackerParams& params, const std::string& value) {
    return setValue(params.min_plate_width, value);
  }

  //-------------------------------------------------------------------------
  bool setDisableGpu(SSDK::CarTrackerParams& params, const std::string& value) {
    return setValue(params.disable_gpu, value);
  }

  //-------------------------------------------------------------------------
  std::map<std::string, std::function<bool(SSDK::CarTrackerParams&, const std::string&)>> s_setters = { {"thread_num", setThreads}, {"plate_max_width", setPlateMaxWidth}, {"plate_min_width", setPlateMinWidth}, {"disable_gpu", setDisableGpu} };

  //-------------------------------------------------------------------------
  std::tuple<bool, std::string, std::string> getTypeAndValue(const std::string& param) {
    auto res = param.find('|');
    if (res != std::string::npos) {
      auto type  = param.substr(0, res);
      auto value = param.substr(res + 1, param.length() - res - 1);
      return std::make_tuple(true, type, value)
    }
    return std::make_tuple(false, "", "");
  }

  //-------------------------------------------------------------------------
  bool execSetting(SSDK::CarTrackerParams& params, const std::string& param) {
    bool result = true;
    try {
      auto elems = getTypeAndValue(param);
      if (std::get<0>(elems)) {
        auto type = std::get<1>(elems);
        auto value = std::get<2>(elems);
        if (s_setters.count(type) > 0) {
          if (!s_setters[type](params, value)) {
            std::cerr << "Setter did't correct work for type \'" << type << "\'.\n";
            result = false;
          }
        }
        else {
          std::cerr << "Can't find setter for \'" << type << "\'.\n";
          result = false;
        }
      }
    }
    catch (std::exception& error) {
      std::cerr << error.what() << std::endl;
      result = false;
    }
    return result;
  }

  //-------------------------------------------------------------------------
  bool setParams(SSDK::CarTrackerParams& params, const std::string& file_name) {
    std::ifstream file(file_name);
    bool result = true;
    if (file.is_open()) {
      std::string line;
      while (std::getline(file, line)) {
        result = result && execSetting(params, line);
      }
      file.close();
    }
    else {
      std::cerr << "Can't open file \'" << filen_name << "\'.\n";
      result = false;
    }
    return result;
  }

}