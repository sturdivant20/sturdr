#include <Eigen/Dense>
#include <iostream>
#include <sstream>

#include "sturdr/nav/clock.hpp"
#include "sturdr/utils/io-tools.hpp"
#include "sturdr/utils/structs-enums.hpp"

int main() {
  // initialize logger
  std::shared_ptr<spdlog::logger> logger = spdlog::default_logger();
  logger->set_level(spdlog::level::trace);
  // logger->set_pattern("%^[%Y-%m-%d %H:%M:%S.%e] [%l] %v %$");

  // intialize sdr struct and generate settings
  sturdr::YamlParser yp("config/gps_l1ca_rcvr.yaml");
  logger->info("yaml file parsed");
  uint64_t ms_to_process = yp.GetVar<uint64_t>("ms_to_process");
  std::string scenario = yp.GetVar<std::string>("scenario");
  std::string in_file = yp.GetVar<std::string>("in_file");
  double reference_pos_x = yp.GetVar<double>("reference_pos_x");
  double reference_pos_y = yp.GetVar<double>("reference_pos_y");
  double reference_pos_z = yp.GetVar<double>("reference_pos_z");
  std::string clock_model = yp.GetVar<std::string>("clock_model");

  // print out some variables
  logger->info("ms_to_process: {}", ms_to_process);
  logger->info("scenario: {}", scenario);
  logger->info("in_file: {}", in_file);
  logger->info("reference_pos_x: {}", reference_pos_x);
  logger->info("reference_pos_y: {}", reference_pos_y);
  logger->info("reference_pos_z: {}", reference_pos_z);

  // parse some data using class
  sturdr::RfDataFile file(in_file);
  const int N = 10;
  // std::vector<int8_t> stream(N);
  Eigen::Vector<int8_t, N> stream;
  file.fread(stream.data(), N);

  std::ostringstream oss;
  oss << "stream: { ";
  for (int i = 0; i < N; i++) {
    oss << static_cast<int>(stream[i]) << ", ";
  }
  std::string str = oss.str();
  str.erase(str.length() - 2);
  std::cout << str << " }\n";

  // Print out some status flags/enums
  std::cout << "Constellation: " << sturdr::GnssSystem::GPS << "\n";
  std::cout << "Signal: " << sturdr::GnssSignal::GPS_L1CA << "\n";
  std::cout << "Measurement: " << (sturdr::MeasurementType::MeasurementType)0b00001011 << "\n";
  std::cout << "Channel State: " << sturdr::ChannelState::ACQUIRING << "\n";
  std::cout << "Tracking Status: " << (sturdr::TrackingFlags::TrackingFlags)0b10010111 << "\n";

  // get a nav clock
  sturdr::NavigationClock clk = sturdr::GetNavClock(clock_model);
  std::cout << "Clock Param: { h0=" << clk.h0 << ", h1=" << clk.h1 << ", h2=" << clk.h2 << " }\n";

  return 0;
}