
#include <spdlog/fmt/ostr.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <Eigen/Dense>
#include <iostream>
#include <sstream>

#include "sturdr/nav/clock.hpp"
#include "sturdr/utils/io-tools.hpp"
#include "sturdr/utils/structs-enums.hpp"

int main() {
  // initialize logger
  std::shared_ptr<spdlog::logger> console = spdlog::stdout_color_mt("sturdr-console");
  console->set_pattern("\033[1;34m[%D %T.%e][%^%l%$\033[1;34m]: \033[0m%v");
  console->set_level(spdlog::level::trace);

  // intialize sdr struct and generate settings
  sturdr::YamlParser yp("config/gps_l1ca_rcvr.yaml");
  console->info("yaml file parsed");
  uint64_t ms_to_process = yp.GetVar<uint64_t>("ms_to_process");
  std::string scenario = yp.GetVar<std::string>("scenario");
  std::string in_file = yp.GetVar<std::string>("in_file");
  double reference_pos_x = yp.GetVar<double>("reference_pos_x");
  double reference_pos_y = yp.GetVar<double>("reference_pos_y");
  double reference_pos_z = yp.GetVar<double>("reference_pos_z");
  std::string clock_model = yp.GetVar<std::string>("clock_model");

  // print out some variables
  console->info("ms_to_process: {}", ms_to_process);
  console->info("scenario: {}", scenario);
  console->info("in_file: {}", in_file);
  console->info("reference_pos_x: {}", reference_pos_x);
  console->info("reference_pos_y: {}", reference_pos_y);
  console->info("reference_pos_z: {}", reference_pos_z);

  // parse some data using class
  sturdr::RfDataFile file(in_file);
  const int N = 10;
  // std::vector<int8_t> stream(N);
  Eigen::Vector<int8_t, N> stream;
  file.fread(stream.data(), N);

  std::ostringstream oss;
  oss << "stream: [ ";
  for (int i = 0; i < N; i++) {
    oss << static_cast<int>(stream[i]) << ", ";
  }
  std::string str = oss.str();
  str.erase(str.length() - 2);
  console->info("{} ]", str);

  // Print out some status flags/enums
  console->trace("Constellation: {}", sturdr::GnssSystem::GPS);
  console->debug("Signal: {}", sturdr::GnssSignal::GPS_L1CA);
  console->info("Measurement: {}", (sturdr::MeasurementType::MeasurementType)0b00001011);
  console->warn("Channel State: {}", sturdr::ChannelState::ACQUIRING);
  console->error("Tracking Status: {}", (sturdr::TrackingFlags::TrackingFlags)0b10010111);

  // get a nav clock
  sturdr::NavigationClock clk = sturdr::GetNavClock(clock_model);
  console->critical("Clock Param: h0={}, h1={}, h2={}", clk.h0, clk.h1, clk.h2);

  return 0;
}