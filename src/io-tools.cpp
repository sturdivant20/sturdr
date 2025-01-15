/**
 * *io-tools.cpp*
 *
 * =======  ========================================================================================
 * @file    sturdr/io-tools.hpp
 * @brief   File tools.
 * @date    December 2024
 * @author  Daniel Sturdivant <sturdivant20@gmail.com>
 * =======  ========================================================================================
 */

#include "sturdr/io-tools.hpp"

#include <filesystem>

namespace sturdr {

//*=== EnsurePathExists ===*
void EnsurePathExists(std::string path) {
  if (!std::filesystem::exists(path)) {
    std::filesystem::create_directory(path);
  }
}

//! ------------------------------------------------------------------------------------------------

// *=== RfDataFile ===*
RfDataFile::RfDataFile(const std::string fname, const std::string fpath)
    : log_{spdlog::get("sturdr-console")} {
  std::stringstream tmp;
  tmp << fpath << "/" << fname;
  fname_ = tmp.str();
  fopen();
}
RfDataFile::RfDataFile(const std::string fname)  //
    : log_{spdlog::get("sturdr-console")}, fname_{fname} {
  fopen();
}

// *=== ~RfDataFile ===*
RfDataFile::~RfDataFile() {
  fclose();
}

//  *=== fopen ===*
bool RfDataFile::fopen() {
  try {
    // reading file
    fid_ = std::ifstream(fname_, std::ios::binary);
    if (fid_.is_open() || fid_.good()) {
      return true;
    }
    return false;
  } catch (std::exception const &e) {
    log_->error("RfDataFile::fopen unable to open file. Error -> {}.", e.what());
    return false;
  }
}

// *=== fclose ===*
bool RfDataFile::fclose() {
  try {
    fid_.close();
    return true;
  } catch (std::exception const &e) {
    log_->error("RfDataFile::fopen unable to close file. Error -> {}.", e.what());
    return false;
  }
}

//! ------------------------------------------------------------------------------------------------

// *=== YamlParser ===*
YamlParser::YamlParser(const std::string fname, const std::string fpath)
    : log_{spdlog::get("sturdr-console")} {
  std::stringstream tmp;
  tmp << fpath << "/" << fname;
  fname_ = tmp.str();

  // parse file
  parse();
}
YamlParser::YamlParser(const std::string fname)
    : fname_{fname}, log_{spdlog::get("sturdr-console")} {
  // parse file
  parse();
}

// *=== ~YamlParser ===*
YamlParser::~YamlParser() {
}

// *=== parse ===*
void YamlParser::parse() {
  try {
    // open file
    std::ifstream fid_(fname_);
    if (!fid_.is_open() || !fid_.good()) {
      log_->error("io-tools.hpp YamlParser unable to open file");
    }

    // read line by line
    std::string line, key, val;
    std::size_t pos;
    while (std::getline(fid_, line)) {
      // Skip lines that are empty or start with '#'
      if (line.empty() || line[0] == '#') {
        continue;
      }

      // ensure there is a keyword and a value
      pos = line.find(":");
      if (pos != std::string::npos) {
        key = line.substr(0, pos);
        val = line.substr(pos + 1);

        // trim whitespace
        key.erase(0, key.find_first_not_of(" \t\n\r\f\v"));
        key.erase(key.find_last_not_of(" \t\n\r\f\v") + 1);
        val.erase(0, val.find_first_not_of(" \t\n\r\f\v"));
        val.erase(val.find_last_not_of(" \t\n\r\f\v") + 1);

        // add to map
        root_[key] = val;
      }
    }

    // close file
    fid_.close();

  } catch (std::exception const &e) {
    log_->error("YamlParser::parse failed. Error -> {}.", e.what());
  }
}

// *=== Exists ===*
bool YamlParser::Exists(const std::string &name) {
  return (root_.find(name) != root_.end());
}

}  // namespace sturdr