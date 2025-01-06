/**
 * *io-tools.hpp*
 *
 * =======  ========================================================================================
 * @file    sturdr/utils/io-tools.hpp
 * @brief   File tools.
 * @date    December 2024
 * @author  Daniel Sturdivant <sturdivant20@gmail.com>
 * =======  ========================================================================================
 */

#pragma once

#ifndef STURDR_IO_TOOLS_HPP
#define STURDR_IO_TOOLS_HPP

#include <spdlog/spdlog.h>

// #include <Eigen/Dense>
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>

namespace sturdr {

/**
 * *=== EnsurePathExists ===*
 * @brief Makes sure directory exists
 * @param path  String containing directory
 */
void EnsurePathExists(std::string path);

//! ------------------------------------------------------------------------------------------------

class RfDataFile {
 public:
  /**
   * *=== RfDataFile ===*
   * @brief constructor
   * @param fname string containing signal file name (if path and fname_ is one string, this
   * is the only input)
   * @param fpath string containing path to signal file
   */
  RfDataFile(const std::string fname, const std::string fpath);
  RfDataFile(const std::string fname);

  /**
   * *=== ~RfDataFile ===*
   * @brief destructor
   */
  ~RfDataFile();

  /**
   * *=== fread ===*
   * @brief read data from file
   * @param buffer destination of data parsed from file
   * @param len    number of samples to read
   * @return True|False based upon reading/parsing success
   */
  template <typename T>
  bool fread(T *buffer, const int len) {
    try {
      if (!fid_.is_open() || fid_.bad()) {
        log_->error("io-tools.hpp RfDataFile::fread bad signal file.");
        return false;
      }
      // read into standard array
      fid_.read(reinterpret_cast<char *>(buffer), len * sizeof(T));
      return true;
    } catch (std::exception const &e) {
      log_->error("io-tools.hpp RfDataFile::fread unable to read file. Error -> {}.", e.what());
      return false;
    }
  };

  /**
   * *=== fseek ===*
   * @brief seek to a specific sample from the beginning of the file
   * @param len    number of samples to skip from beginning of file
   * @return True|False based upon seeking success
   */
  template <typename T>
  bool fseek(const int len) {
    try {
      if (!fid_.is_open() || fid_.bad()) {
        log_->error("io-tools.hpp RfDataFile::fseek bad signal file.");
        return false;
      }
      // seek to sample
      fid_.seekg(len * sizeof(T), fid_.beg);
      return true;
    } catch (std::exception const &e) {
      log_->error("io-tools.hpp RfDataFile::fseek unable to seek in file. Error -> {}.", e.what());
      return false;
    }
  };

  /**
   * *=== ftell ===*
   * @brief retrieve sample number from the beginning of the file
   * @param location sample index from beginning of file
   * @return True|False based upon telling success
   */
  template <typename T>
  bool ftell(int &location) {
    try {
      if (!fid_.is_open() || fid_.bad()) {
        log_->error("io-tools.hpp RfDataFile::ftell bad signal file.");
        return false;
      }
      // tell sample
      location = fid_.tellg() / sizeof(T);
      return true;
    } catch (std::exception const &e) {
      log_->error("io-tools.hpp RfDataFile::ftell unable to tell in file. Error -> {}.", e.what());
      return false;
    }
  };
  template <typename T>
  int ftell() {
    int location;
    if (ftell<T>(location)) {
      return location;
    } else {
      return -1;
    }
  };

  /**
   *  *=== fopen ===*
   * @brief attempts to open signal file under specified fname_
   * @return True|False based upon opening success
   */
  bool fopen();

  /**
   * *=== fclose ===*
   * @brief attempts to close active file identifier
   * @return True|False based upon closing success
   */
  bool fclose();

 private:
  std::shared_ptr<spdlog::logger> log_;
  std::string fname_;
  std::ifstream fid_;
};  // end class RfDataFile

//! ------------------------------------------------------------------------------------------------

class YamlParser {
 public:
  /**
   * *=== YamlParser ===*
   * @brief Constructor
   * @param fname string containing yaml file name
   * @param fpath (optional) string containing yaml file path (directory)
   */
  YamlParser(const std::string fname, const std::string fpath);
  YamlParser(const std::string fname);

  /**
   * *=== ~YamlParser ===*
   * @brief destructor
   */
  ~YamlParser();

  /**
   * *=== parse ===*
   * @brief parses the yaml file into a map of string-(string-string)
   *        -> only supports 1 sub-layer
   *        -> assumes the separators to be either ': ' for key-value pairs OR ':\n' for major
   *           groupings distinctions
   */
  void parse();

  /**
   * *=== Exists ===*
   * @brief Query wether paramater name exists
   * @returns True|False based on parameter existence
   */
  bool Exists(const std::string &name);

  /**
   * *=== GetVar ===*
   * @brief Get variable as specified type
   * @param val   Item to place value into
   * @param name  Name of variable to grab
   * @return True|False based on successful extraction
   */
  template <typename T>
  bool GetVar(T &val, const std::string name) {
    try {
      // make sure variable exists
      if (Exists(name)) {
        std::istringstream iss(root_[name]);
        iss >> val;
        return true;
      } else {
        log_->warn("YamlParser::GetVar no keyword '{}' in file!", name);
        return false;
      }
    } catch (std::exception const &e) {
      log_->error("YamlParser::GetVar failed. Error -> {}.", e.what());
      return false;
    }
  };
  template <typename T>
  T GetVar(const std::string name) {
    T var;
    GetVar(var, name);
    return var;
  };

 private:
  std::string fname_;
  std::map<std::string, std::string> root_;
  std::shared_ptr<spdlog::logger> log_;

};  // end class YamlParser

}  // end namespace sturdr

#endif