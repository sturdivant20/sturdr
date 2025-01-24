
#include <spdlog/async.h>
#include <spdlog/fmt/ostr.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/stopwatch.h>

#include <Eigen/Dense>
#include <complex>
#include <functional>
#include <map>
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
#include "sturdr/data-type-adapters.hpp"
#include "sturdr/fftw-wrapper.hpp"
#include "sturdr/structs-enums.hpp"

using NavQueue = sturdr::ConcurrentQueue<sturdr::ChannelNavPacket>;
using EphemQueue = sturdr::ConcurrentQueue<sturdr::ChannelEphemPacket>;
using Barrier = sturdr::ConcurrentBarrier;
std::mutex prn_mtx;

// prn ids in use
std::map<uint8_t, bool> prns_in_use;
int prn_ptr = 1;

void GetNewPrn(uint8_t &prn) {
  std::unique_lock<std::mutex> lock(prn_mtx);

  // remove prn from log
  prns_in_use[prn] = false;

  // search for next available prn
  while (prns_in_use[prn_ptr]) {
    prn_ptr = prn_ptr % 32 + 1;
  }

  // log new prn
  prn = prn_ptr;
  prns_in_use[prn_ptr] = true;
  prn_ptr = prn_ptr % 32 + 1;
  // std::cout << "prn_ptr_: " << (int)prn_ptr << "\n";
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
  console->debug("Barriers created!");

  // initialize file reader
  sturdio::BinaryFile file(conf.general.in_file);
  // Eigen::Vector<int8_t, Eigen::Dynamic> in_stream(shm_samp_write_size);
  std::vector<int8_t> in_stream(shm_samp_write_size);
  console->debug("RfDataFile created!");

  // initialize acquisition matrices
  int samp_per_ms = conf.rfsignal.samp_freq / 1000;
  int n_dopp_bins = static_cast<uint64_t>(
      2.0 * conf.acquisition.doppler_range / conf.acquisition.doppler_step + 1.0);
  sturdr::FftPlans fftw_plans{
      sturdr::Create1dFftPlan(samp_per_ms, true),
      sturdr::Create1dFftPlan(samp_per_ms, false),
      sturdr::CreateManyFftPlanColWise(samp_per_ms, n_dopp_bins, true),
      sturdr::CreateManyFftPlanColWise(samp_per_ms, n_dopp_bins, false)};

  // runtime boolean
  std::shared_ptr<bool> still_running = std::make_shared<bool>(true);

  // create channels
  for (uint8_t i = 1; i <= 32; i++) {
    prns_in_use.insert({i, false});
  }
  std::vector<sturdr::ChannelGpsL1ca> chs;
  chs.reserve(conf.rfsignal.max_channels);
  std::function<void(uint8_t &)> tmp = std::bind(&GetNewPrn, std::placeholders::_1);
  for (uint8_t channel_num = 1; channel_num <= (uint8_t)conf.rfsignal.max_channels; channel_num++) {
    chs.emplace_back(
        conf,
        channel_num,
        still_running,
        shm,
        start_barrier,
        eph_queue,
        nav_queue,
        fftw_plans,
        tmp);
    chs[channel_num - 1].Start();
  }
  std::cout << "here main 1\n";

  // initial read signal data
  file.fread(in_stream.data(), shm_samp_write_size);
  std::cout << "here main 2\n";
  // for (uint64_t j = 0; j < shm_samp_write_size; j++) {
  //   (*shm)((shm_ptr + j) % shm_samp_chunk_size) =
  //   static_cast<std::complex<double>>(in_stream[j]);
  // }
  sturdr::ByteToIDouble(
      in_stream.data(), shm->segment(shm_ptr, shm_samp_write_size).data(), shm_samp_write_size);
  shm_ptr += shm_samp_write_size;
  shm_ptr %= shm_samp_chunk_size;
  std::cout << "here main 3\n";

  // run channel
  spdlog::stopwatch sw;
  for (int i = 0; i < (int)conf.general.ms_to_process + 1; i += (int)conf.general.ms_read_size) {
    // ready to process data
    // std::cout << "here main 4\n";
    start_barrier->Wait();

    // read signal data
    file.fread(in_stream.data(), shm_samp_write_size);
    // for (uint64_t j = 0; j < shm_samp_write_size; j++) {
    //   (*shm)((shm_ptr + j) % shm_samp_chunk_size) =
    //   static_cast<std::complex<double>>(in_stream[j]);
    // }
    sturdr::ByteToIDouble(
        in_stream.data(), shm->segment(shm_ptr, shm_samp_write_size).data(), shm_samp_write_size);
    shm_ptr += shm_samp_write_size;
    shm_ptr %= shm_samp_chunk_size;

    // check for screen printouts
    if (i % 1000 == 0) {
      console->info("File time: {:.3f} s ... Processing Time: {:.3f} s", (float)i / 1000.0, sw);
    }
  }
  *still_running = false;
  console->info("test_channel.cpp still_running = {}", *still_running);
  start_barrier->NotifyComplete();

  // join channels
  for (int channel_num = 0; channel_num < (int)conf.rfsignal.max_channels; channel_num++) {
    chs[channel_num].Join();
  }

  return 0;
}