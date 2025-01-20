
#include <spdlog/async.h>
#include <spdlog/fmt/ostr.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/stopwatch.h>

#include <Eigen/Dense>
#include <complex>
#include <memory>
#include <mutex>
#include <numeric>
#include <satutils/code-gen.hpp>
#include <sturdio/binary-file.hpp>
#include <sturdio/yaml-parser.hpp>
#include <vector>

#include "satutils/gnss-constants.hpp"
#include "sturdr/acquisition.hpp"
#include "sturdr/channel-gps-l1ca.hpp"
#include "sturdr/concurrent-barrier.hpp"
#include "sturdr/concurrent-queue.hpp"
#include "sturdr/structs-enums.hpp"

using NavQueue = sturdr::ConcurrentQueue<sturdr::ChannelNavPacket>;
using EphemQueue = sturdr::ConcurrentQueue<sturdr::ChannelEphemPacket>;
using Barrier = sturdr::ConcurrentBarrier;
std::mutex mtx;

// prn ids in use
std::vector<uint8_t> prns(32);
std::vector<uint8_t> prns_in_use;
int prn_idx = 0;

void GetNewPrn(uint8_t &prn, std::array<bool, 1023> &code) {
  // spdlog::get("sturdr-console")->warn("prn a: {}, prn_idx: {}", prn, prn_idx);
  std::unique_lock<std::mutex> lock(mtx);

  // remove prn from log
  if (prns_in_use.size() > 0) {
    auto i = std::find(prns_in_use.begin(), prns_in_use.end(), prn);
    if (i != prns_in_use.end()) {
      prns_in_use.erase(i);
    }
  }

  // search for next available prn
  while (std::find(prns_in_use.begin(), prns_in_use.end(), prns[prn_idx]) != prns_in_use.end()) {
    prn_idx += 1;
    prn_idx %= 32;
  }

  // log new prn
  prn = prns[prn_idx];
  // spdlog::get("sturdr-console")->warn("prn b: {}, prn_idx: {}", prn, prn_idx);
  prns_in_use.push_back(prn);
  // spdlog::get("sturdr-console")->warn("prns_in_use: {}", fmt::join(prns_in_use, ", "));
  prn_idx += 1;
  prn_idx %= 32;
}

int main(int argc, char *argv[]) {
  std::string yaml_filename;
  if (argc > 1) {
    yaml_filename = argv[1];
  } else {
    yaml_filename = "config/gps_l1ca_rcvr.yaml";
  }

  // initialize logger
  // std::shared_ptr<spdlog::logger> console = spdlog::stdout_color_mt("sturdr-console");
  std::shared_ptr<spdlog::logger> console =
      spdlog::stdout_color_mt<spdlog::async_factory>("sturdr-console");
  console->set_pattern("\033[1;34m[%D %T.%e][%^%l%$\033[1;34m]: \033[0m%v");

  // config
  console->info("Yaml File: {}", yaml_filename);
  sturdr::Config conf;
  sturdio::YamlParser yp(yaml_filename);
  conf.general.scenario = yp.GetVar<std::string>("scenario");
  conf.general.in_file = yp.GetVar<std::string>("in_file");
  conf.general.out_folder = yp.GetVar<std::string>("out_folder");
  conf.general.log_level = spdlog::level::from_str(yp.GetVar<std::string>("log_level"));
  conf.general.ms_to_process = yp.GetVar<uint64_t>("ms_to_process");
  conf.general.ms_chunk_size = yp.GetVar<uint16_t>("ms_chunk_size");
  conf.general.ms_read_size = yp.GetVar<uint16_t>("ms_read_size");
  conf.general.reference_pos_x = yp.GetVar<double>("reference_pos_x");
  conf.general.reference_pos_y = yp.GetVar<double>("reference_pos_y");
  conf.general.reference_pos_z = yp.GetVar<double>("reference_pos_z");
  conf.rfsignal.samp_freq = yp.GetVar<double>("samp_freq");
  conf.rfsignal.intmd_freq = yp.GetVar<double>("intmd_freq");
  conf.rfsignal.is_complex = yp.GetVar<bool>("is_complex");
  conf.rfsignal.bit_depth = yp.GetVar<uint16_t>("bit_depth");
  conf.rfsignal.signals = yp.GetVar<std::string>("signals");
  conf.rfsignal.max_channels = yp.GetVar<uint16_t>("max_channels");
  conf.acquisition.threshold = yp.GetVar<double>("threshold");
  conf.acquisition.doppler_range = yp.GetVar<double>("doppler_range");
  conf.acquisition.doppler_step = yp.GetVar<double>("doppler_step");
  conf.acquisition.num_coh_per = yp.GetVar<uint16_t>("num_coh_per");
  conf.acquisition.num_noncoh_per = yp.GetVar<uint16_t>("num_noncoh_per");
  conf.acquisition.max_failed_attempts = yp.GetVar<uint16_t>("max_failed_attempts");
  conf.tracking.min_converg_time_ms = yp.GetVar<uint16_t>("min_converg_time_ms");
  conf.tracking.tap_epl_wide = yp.GetVar<double>("tap_epl_wide");
  conf.tracking.tap_epl_standard = yp.GetVar<double>("tap_epl");
  conf.tracking.tap_epl_narrow = yp.GetVar<double>("tap_epl_narrow");
  conf.tracking.dll_bw_wide = yp.GetVar<double>("dll_bandwidth_wide");
  conf.tracking.pll_bw_wide = yp.GetVar<double>("pll_bandwidth_wide");
  conf.tracking.fll_bw_wide = yp.GetVar<double>("fll_bandwidth_wide");
  conf.tracking.dll_bw_standard = yp.GetVar<double>("dll_bandwidth");
  conf.tracking.pll_bw_standard = yp.GetVar<double>("pll_bandwidth");
  conf.tracking.fll_bw_standard = yp.GetVar<double>("fll_bandwidth");
  conf.tracking.dll_bw_narrow = yp.GetVar<double>("dll_bandwidth_narrow");
  conf.tracking.pll_bw_narrow = yp.GetVar<double>("pll_bandwidth_narrow");
  conf.tracking.fll_bw_narrow = yp.GetVar<double>("fll_bandwidth_narrow");
  conf.navigation.use_psr = yp.GetVar<bool>("use_psr");
  conf.navigation.use_doppler = yp.GetVar<bool>("use_doppler");
  conf.navigation.use_adr = yp.GetVar<bool>("use_adr");
  conf.navigation.use_cno = yp.GetVar<bool>("use_cno");
  conf.navigation.do_vt = yp.GetVar<bool>("do_vt");
  conf.navigation.meas_freq = yp.GetVar<uint16_t>("meas_freq");
  conf.navigation.clock_model = yp.GetVar<std::string>("clock_model");
  conf.navigation.process_std = yp.GetVar<double>("process_std");
  conf.navigation.nominal_transit_time = yp.GetVar<double>("nominal_transit_time");
  console->set_level(conf.general.log_level);
  // console->warn("scenario: {}", conf.general.scenario);
  // console->warn("ms_to_process: {}", conf.general.ms_to_process);
  // console->warn("ms_chunk_size: {}", conf.general.ms_chunk_size);
  // console->warn("ms_read_size: {}", conf.general.ms_read_size);
  // console->warn("in_file: {}", conf.general.in_file);
  // console->warn("out_folder: {}", conf.general.out_folder);
  // console->warn("reference_pos_x: {}", conf.general.reference_pos_x);
  // console->warn("reference_pos_y: {}", conf.general.reference_pos_y);
  // console->warn("reference_pos_z: {}", conf.general.reference_pos_z);
  // console->warn("log_level: {}", conf.general.log_level);
  // console->warn("samp_freq: {}", conf.rfsignal.samp_freq);
  // console->warn("intmd_freq: {}", conf.rfsignal.intmd_freq);
  // console->warn("is_complex: {}", conf.rfsignal.is_complex);
  // console->warn("bit_depth: {}", conf.rfsignal.bit_depth);
  // console->warn("signals: {}", conf.rfsignal.signals);
  // console->warn("max_channels: {}", conf.rfsignal.max_channels);
  // console->warn("doppler_range: {}", conf.acquisition.doppler_range);
  // console->warn("doppler_step: {}", conf.acquisition.doppler_step);
  // console->warn("num_coh_per: {}", conf.acquisition.num_coh_per);
  // console->warn("num_noncoh_per: {}", conf.acquisition.num_noncoh_per);
  // console->warn("threshold: {}", conf.acquisition.threshold);
  // console->warn("min_converg_time_ms: {}", conf.tracking.min_converg_time_ms);
  // console->warn("tap_epl_wide: {}", conf.tracking.tap_epl_wide);
  // console->warn("tap_epl: {}", conf.tracking.tap_epl_standard);
  // console->warn("tap_epl_narrow: {}", conf.tracking.tap_epl_narrow);
  // console->warn("pll_bandwidth_wide: {}", conf.tracking.pll_bw_wide);
  // console->warn("fll_bandwidth_wide: {}", conf.tracking.fll_bw_wide);
  // console->warn("dll_bandwidth_wide: {}", conf.tracking.dll_bw_wide);
  // console->warn("pll_bandwidth: {}", conf.tracking.pll_bw_standard);
  // console->warn("fll_bandwidth: {}", conf.tracking.fll_bw_standard);
  // console->warn("dll_bandwidth: {}", conf.tracking.dll_bw_standard);
  // console->warn("pll_bandwidth_narrow: {}", conf.tracking.pll_bw_narrow);
  // console->warn("fll_bandwidth_narrow: {}", conf.tracking.fll_bw_narrow);
  // console->warn("dll_bandwidth_narrow: {}", conf.tracking.dll_bw_narrow);
  // console->warn("meas_freq: {}", conf.navigation.meas_freq);
  // console->warn("process_std: {}", conf.navigation.process_std);
  // console->warn("clock_model: {}", conf.navigation.clock_model);
  // console->warn("nominal_transit_time: {}", conf.navigation.nominal_transit_time);
  // console->warn("use_psr: {}", conf.navigation.use_psr);
  // console->warn("use_doppler: {}", conf.navigation.use_doppler);
  // console->warn("use_adr: {}", conf.navigation.use_adr);
  // console->warn("use_cno: {}", conf.navigation.use_cno);
  // console->warn("do_vt: {}", conf.navigation.do_vt);

  // initialize shared memory
  uint64_t shm_ptr = 0;
  uint64_t shm_samp_chunk_size = conf.general.ms_chunk_size * conf.rfsignal.samp_freq / 1000;
  uint64_t shm_samp_write_size = conf.general.ms_read_size * conf.rfsignal.samp_freq / 1000;
  std::shared_ptr<Eigen::VectorXcd> shm =
      std::make_shared<Eigen::VectorXcd>(Eigen::VectorXcd::Zero(shm_samp_chunk_size));
  console->info("shm_samp_write_size = {}", shm_samp_write_size);
  console->info("shm_samp_chunk_size = {}", shm_samp_chunk_size);

  // initialize thread safe queues
  std::shared_ptr<NavQueue> nav_queue = std::make_shared<NavQueue>();
  std::shared_ptr<EphemQueue> eph_queue = std::make_shared<EphemQueue>();
  console->debug("Queues created!");

  // initialize thread syncronization barriers
  std::shared_ptr<Barrier> start_barrier =
      std::make_shared<Barrier>(conf.rfsignal.max_channels + 1);
  std::shared_ptr<Barrier> end_barrier = std::make_shared<Barrier>(conf.rfsignal.max_channels + 1);
  console->debug("Barriers created!");

  // initialize file reader
  sturdio::BinaryFile file(conf.general.in_file);
  Eigen::Vector<int8_t, Eigen::Dynamic> in_stream(shm_samp_write_size);
  console->debug("RfDataFile created!");

  // initialize acquisition matrices
  std::array<std::array<bool, 1023>, 32> codes;
  for (int i = 0; i < 32; i++) {
    satutils::CodeGenCA(codes[i], i + 1);
  }
  sturdr::AcquisitionSetup acq_setup = sturdr::InitAcquisitionMatrices(
      codes,
      conf.acquisition.doppler_range,
      conf.acquisition.doppler_step,
      conf.rfsignal.samp_freq,
      satutils::GPS_CA_CODE_RATE<>,
      conf.rfsignal.intmd_freq);

  // runtime boolean
  std::shared_ptr<bool> still_running = std::make_shared<bool>(true);

  // create channels
  // uint8_t prn_to_use[8] = {1, 7, 13, 14, 17, 19, 21, 30};
  std::iota(prns.begin(), prns.end(), 1);
  std::vector<sturdr::GpsL1caChannel> chs;
  chs.reserve(conf.rfsignal.max_channels);
  for (int channel_num = 0; channel_num < (int)conf.rfsignal.max_channels; channel_num++) {
    // chs.push_back(sturdr::GpsL1caChannel(
    //     conf, p, shm, channel_queue, nav_queue, start_barrier, end_barrier, channel_num));
    // GetNewPrn(current_prn);
    prns_in_use.push_back(channel_num + 1);
    chs.emplace_back(
        conf,
        acq_setup,
        shm,
        nav_queue,
        eph_queue,
        start_barrier,
        end_barrier,
        channel_num,
        still_running,
        &GetNewPrn);
    chs[channel_num].SetSatellite(channel_num + 1);
    console->info("Channel {} created and set to GPS{}!", channel_num, channel_num + 1);
    chs[channel_num].start();
  }

  // initial read signal data
  file.fread(in_stream.data(), shm_samp_write_size);
  for (uint64_t j = 0; j < shm_samp_write_size; j++) {
    (*shm)((shm_ptr + j) % shm_samp_chunk_size) = static_cast<std::complex<double>>(in_stream(j));
  }
  shm_ptr += shm_samp_write_size;
  shm_ptr %= shm_samp_chunk_size;

  // run channel
  spdlog::stopwatch sw;
  for (int i = 0; i < (int)conf.general.ms_to_process + 1; i += (int)conf.general.ms_read_size) {
    // ready to process data
    start_barrier->Wait();

    // read signal data
    file.fread(in_stream.data(), shm_samp_write_size);
    for (uint64_t j = 0; j < shm_samp_write_size; j++) {
      (*shm)((shm_ptr + j) % shm_samp_chunk_size) = static_cast<std::complex<double>>(in_stream(j));
    }
    shm_ptr += shm_samp_write_size;
    shm_ptr %= shm_samp_chunk_size;

    // check for screen printouts
    if (i % 1000 == 0) {
      console->info("File time: {:.3f} s ... Processing Time: {:.3f} s", (float)i / 1000.0, sw);
    }

    // channels finished
    end_barrier->Wait();
  }
  *still_running = false;
  console->info("test_channel.cpp still_running = {}", *still_running);

  // join channels
  for (int channel_num = 0; channel_num < (int)conf.rfsignal.max_channels; channel_num++) {
    chs[channel_num].join();
  }

  return 0;
}