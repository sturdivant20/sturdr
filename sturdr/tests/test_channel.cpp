
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include <Eigen/Dense>
#include <complex>
#include <memory>
#include <vector>

#include "sturdr/channel/gps-l1ca-channel.hpp"
#include "sturdr/utils/concurrent-barrier.hpp"
#include "sturdr/utils/concurrent-queue.hpp"
#include "sturdr/utils/fftw-wrapper.hpp"
#include "sturdr/utils/io-tools.hpp"
#include "sturdr/utils/structs-enums.hpp"

using ChannelQueue = sturdr::ConcurrentQueue<sturdr::ChannelPacket>;
using NavQueue = sturdr::ConcurrentQueue<sturdr::NavPacket>;
using Barrier = sturdr::ConcurrentBarrier;

int main() {
  // initialize logger
  std::shared_ptr<spdlog::logger> console = spdlog::stdout_color_mt("sturdr-console");
  console->set_pattern("\033[1;34m[%D %T.%e][%^%l%$\033[1;34m]: \033[0m%v");
  console->set_level(spdlog::level::info);

  // config
  sturdr::Config conf;
  conf.general.scenario = "test_channel";
  conf.general.in_file = "../sturdr/rfdata/class_ifen_8bit_20e6_if_5000445.88565834.bin";
  conf.general.out_folder = "./results";
  conf.general.log_level = spdlog::level::trace;
  conf.general.ms_to_process = 10000;
  conf.general.ms_chunk_size = 100;
  conf.general.ms_read_size = 20;
  conf.general.reference_pos_x = 422596.629;
  conf.general.reference_pos_y = -5362864.287;
  conf.general.reference_pos_z = 3415493.797;
  conf.rfsignal.samp_freq = 20e6;
  conf.rfsignal.intmd_freq = 5000445.88565834;
  conf.rfsignal.is_complex = false;
  conf.rfsignal.bit_depth = 8;
  conf.rfsignal.signals = "gps_l1ca";
  conf.rfsignal.max_channels = 7;
  conf.acquisition.threshold = 12.0;
  conf.acquisition.doppler_range = 5000.0;
  conf.acquisition.doppler_step = 100.0;
  conf.acquisition.num_coh_per = 1;
  conf.acquisition.num_noncoh_per = 5;
  conf.tracking.min_converg_time_ms = 150;
  conf.tracking.tap_epl_wide = 0.5;
  conf.tracking.tap_epl_standard = 0.25;
  conf.tracking.tap_epl_narrow = 0.1;
  conf.tracking.dll_bw_wide = 1.0;
  conf.tracking.pll_bw_wide = 15.0;
  conf.tracking.fll_bw_wide = 5.0;
  conf.tracking.dll_bw_standard = 0.5;
  conf.tracking.pll_bw_standard = 10.0;
  conf.tracking.fll_bw_standard = 1.0;
  conf.tracking.dll_bw_narrow = 0.1;
  conf.tracking.pll_bw_narrow = 6.0;
  conf.tracking.fll_bw_narrow = 0.1;
  conf.navigation.use_psr = true;
  conf.navigation.use_doppler = true;
  conf.navigation.use_adr = false;
  conf.navigation.use_cno = true;
  conf.navigation.do_vt = false;
  conf.navigation.meas_freq = 50;
  conf.navigation.clock_model = "low_quality_tcxo";
  conf.navigation.process_std = 1.0;
  conf.navigation.nominal_transit_time = 0.068802;

  // initialize shared memory
  uint64_t shm_ptr = 0;
  uint64_t shm_samp_chunk_size = conf.general.ms_chunk_size * conf.rfsignal.samp_freq / 1000;
  uint64_t shm_samp_write_size = conf.general.ms_read_size * conf.rfsignal.samp_freq / 1000;
  std::shared_ptr<Eigen::VectorXcd> shm =
      std::make_shared<Eigen::VectorXcd>(Eigen::VectorXcd::Zero(shm_samp_chunk_size));
  console->info("shm_samp_write_size = {}", shm_samp_write_size);
  console->info("shm_samp_chunk_size = {}", shm_samp_chunk_size);

  // initialize thread safe queues
  std::shared_ptr<ChannelQueue> channel_queue = std::make_shared<ChannelQueue>();
  std::shared_ptr<NavQueue> nav_queue = std::make_shared<NavQueue>();
  console->debug("Queues created!");

  // initialize thread syncronization barriers
  std::shared_ptr<Barrier> start_barrier =
      std::make_shared<Barrier>(conf.rfsignal.max_channels + 1);
  std::shared_ptr<Barrier> end_barrier = std::make_shared<Barrier>(conf.rfsignal.max_channels + 1);
  console->debug("Barriers created!");

  // initialize file reader
  sturdr::RfDataFile file(conf.general.in_file);
  Eigen::Vector<int8_t, Eigen::Dynamic> in_stream(shm_samp_write_size);
  console->debug("RfDataFile created!");

  // create fft plans (shared across all threads)
  int samp_per_ms = static_cast<int>(conf.rfsignal.samp_freq / 1000.0);
  int n_dopp_bins =
      static_cast<int>(2.0 * conf.acquisition.doppler_range / conf.acquisition.doppler_step + 1.0);
  sturdr::SturdrFftPlans p;
  p.fft = sturdr::Create1dFftPlan(samp_per_ms, true);
  p.ifft = sturdr::Create1dFftPlan(samp_per_ms, false);
  p.fft_many = sturdr::CreateManyFftPlan(n_dopp_bins, samp_per_ms, true);
  p.ifft_many = sturdr::CreateManyFftPlan(n_dopp_bins, samp_per_ms, false);
  console->debug("FFT plans created!");

  // runtime boolean
  std::shared_ptr<bool> still_running = std::make_shared<bool>(true);

  // create channel
  uint8_t prn[7] = {1, 7, 14, 17, 19, 21, 30};
  std::vector<sturdr::GpsL1caChannel> chs;
  chs.reserve(conf.rfsignal.max_channels);
  for (int channel_num = 0; channel_num < (int)conf.rfsignal.max_channels; channel_num++) {
    // chs.push_back(sturdr::GpsL1caChannel(
    //     conf, p, shm, channel_queue, nav_queue, start_barrier, end_barrier, channel_num));
    chs.emplace_back(
        conf,
        p,
        shm,
        channel_queue,
        nav_queue,
        start_barrier,
        end_barrier,
        channel_num,
        still_running);
    chs[channel_num].SetSatellite(prn[channel_num]);
    console->info("Channel {} created and set to GPS{}!", channel_num, prn[channel_num]);
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