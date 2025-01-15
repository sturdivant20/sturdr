#include <spdlog/fmt/ostr.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <Eigen/Dense>
#include <cmath>
#include <navtools/constants.hpp>
#include <navtools/frames.hpp>
#include <satutils/ephemeris.hpp>
#include <satutils/gnss-constants.hpp>
#include <sturdins/least-squares.hpp>

#include "sturdr/discriminator.hpp"

template <typename T>
struct fmt::formatter<T, std::enable_if_t<std::is_base_of_v<Eigen::DenseBase<T>, T>, char>>
    : ostream_formatter {};

void gen_nav_data(
    Eigen::Vector<satutils::KeplerEphem<double>, 7> &eph,
    Eigen::Vector<double, 7> &ToW,
    Eigen::Vector<double, 7> &CodePhase,
    Eigen::Vector<double, 7> &Doppler,
    Eigen::Vector<double, 7> &cno) {
  satutils::KeplerElements<double> e;
  for (int i = 0; i < 7; i++) {
    switch (i) {
      case 0:
        // i == 0: GPS1
        ToW(i) = 413658.0;
        cno(i) = std::pow(10.0, 47.81385205631905 / 10.0);
        Doppler(i) = 1791.6270479363234;
        CodePhase(i) = 11421.606237540786;

        e.iode = 86.0;
        e.iodc = 86.0;
        e.toe = 417600.0;
        e.toc = 417600.0;
        e.tgd = 4.6566128730773926e-09;
        e.af2 = 0.0;
        e.af1 = -3.751665644813329e-12;
        e.af0 = 0.00020215753465890884;
        e.e = 0.012527465354651213;
        e.sqrtA = 5153.652446746826;
        e.deltan = 3.991594837375442e-09;
        e.m0 = 0.4141951882521321;
        e.omega0 = -2.6390625759606716;
        e.omega = 0.9415036207877718;
        e.omegaDot = -8.228914195877791e-09;
        e.i0 = 0.9897954243352285;
        e.iDot = -3.893019302737323e-11;
        e.cuc = 2.0042061805725098e-06;
        e.cus = 3.2354146242141724e-06;
        e.cic = -7.82310962677002e-08;
        e.cis = 1.3224780559539795e-07;
        e.crc = 329.78125;
        e.crs = 34.625;
        e.ura = 0;
        e.health = 0;
        eph(i).SetEphem(e);
        break;
      case 1:
        // i == 1: GPS7
        ToW(i) = 413658.0;
        cno(i) = std::pow(10.0, 45.636383357667995 / 10.0);
        Doppler(i) = -473.89562817800544;
        CodePhase(i) = 8209.10940035235;

        e.iode = 42.0;
        e.iodc = 42.0;
        e.toe = 417600.0;
        e.toc = 417600.0;
        e.tgd = -1.1175870895385742e-08;
        e.af2 = 0.0;
        e.af1 = -7.73070496506989e-12;
        e.af0 = 0.00022176513448357582;
        e.e = 0.01688421959988773;
        e.sqrtA = 5153.754594802856;
        e.deltan = 5.156643366323071e-09;
        e.m0 = -0.17767922733872102;
        e.omega0 = 0.4888296523228508;
        e.omega = -2.214549307999104;
        e.omegaDot = -8.492496603714501e-09;
        e.i0 = 0.950720208032768;
        e.iDot = 2.3393831589843546e-10;
        e.cuc = 1.2312084436416626e-06;
        e.cus = 7.072463631629944e-06;
        e.cic = -5.029141902923584e-08;
        e.cis = 2.1979212760925293e-07;
        e.crc = 241.53125;
        e.crs = 20.53125;
        e.ura = 0;
        e.health = 0;
        eph(i).SetEphem(e);
        break;
      case 2:
        // i == 1: GPS14
        ToW(i) = 413658.0;
        cno(i) = std::pow(10.0, 45.55573223941419 / 10.0);
        Doppler(i) = 3587.798847894308;
        CodePhase(i) = 11324.089435418902;

        e.iode = 201.0;
        e.iodc = 457.0;
        e.toe = 417600.0;
        e.toc = 417600.0;
        e.tgd = -7.916241884231567e-09;
        e.af2 = 0.0;
        e.af1 = 1.0231815394945443e-11;
        e.af0 = -2.1620653569698334e-06;
        e.e = 0.0027978061698377132;
        e.sqrtA = 5153.666526794434;
        e.deltan = 4.9441345144764e-09;
        e.m0 = -0.9309618696815247;
        e.omega0 = 1.5659970309306777;
        e.omega = -3.0740762061501883;
        e.omegaDot = -8.105694777580142e-09;
        e.i0 = 0.949123931804313;
        e.iDot = -3.507288949805552e-10;
        e.cuc = 1.4360994100570679e-06;
        e.cus = 7.890164852142334e-06;
        e.cic = 6.146728992462158e-08;
        e.cis = -1.862645149230957e-09;
        e.crc = 221.46875;
        e.crs = 28.15625;
        e.ura = 0;
        e.health = 0;
        eph(i).SetEphem(e);
        break;
      case 3:
        // i == 3: GPS17
        ToW(i) = 413658.0;
        cno(i) = std::pow(10.0, 46.2857705159281 / 10.0);
        Doppler(i) = 3279.7089666038046;
        CodePhase(i) = 10902.80528025291;

        e.iode = 95.0;
        e.iodc = 95.0;
        e.toe = 417600.0;
        e.toc = 417600.0;
        e.tgd = -1.1175870895385742e-08;
        e.af2 = 0.0;
        e.af1 = 2.0463630789890885e-12;
        e.af0 = 0.0007058251649141312;
        e.e = 0.013569202041253448;
        e.sqrtA = 5153.734893798828;
        e.deltan = 4.035882396415757e-09;
        e.m0 = 2.5834869705907755;
        e.omega0 = 2.642788161637698;
        e.omega = -1.4142874770588107;
        e.omegaDot = -7.60138805689527e-09;
        e.i0 = 0.975305087634542;
        e.iDot = -2.239378993409451e-10;
        e.cuc = 5.0067901611328125e-06;
        e.cus = 7.33695924282074e-06;
        e.cic = 1.993030309677124e-07;
        e.cis = -5.21540641784668e-08;
        e.crc = 239.875;
        e.crs = 93.84375;
        e.ura = 0;
        e.health = 0;
        eph(i).SetEphem(e);
        break;
      case 4:
        // i == 4: GPS19
        ToW(i) = 413658.0;
        cno(i) = std::pow(10.0, 43.99677860992774 / 10.0);
        Doppler(i) = 5023.396062348471;
        CodePhase(i) = 3954.6679405210402;

        e.iode = 48.0;
        e.iodc = 48.0;
        e.toe = 417600.0;
        e.toc = 417600.0;
        e.tgd = -1.5366822481155396e-08;
        e.af2 = 0.0;
        e.af1 = 5.115907697472721e-12;
        e.af0 = 0.00029091350734233856;
        e.e = 0.009184657246805727;
        e.sqrtA = 5153.615423202515;
        e.deltan = 4.096956368963288e-09;
        e.m0 = -1.5414014968965324;
        e.omega0 = 2.6873690028372486;
        e.omega = 2.2537891669538777;
        e.omegaDot = -7.92890169915308e-09;
        e.i0 = 0.9743497831108455;
        e.iDot = -5.928818387655006e-11;
        e.cuc = 5.358830094337463e-06;
        e.cus = 7.88085162639618e-06;
        e.cic = -1.825392246246338e-07;
        e.cis = -5.587935447692871e-09;
        e.crc = 237.53125;
        e.crs = 102.09375;
        e.ura = 0;
        e.health = 0;
        eph(i).SetEphem(e);
        break;
      case 5:
        // i == 5: GPS21
        ToW(i) = 413658.0;
        cno(i) = std::pow(10.0, 45.96162013805926 / 10.0);
        Doppler(i) = -340.9465647641514;
        CodePhase(i) = 4235.67154014668;

        e.iode = 1.0;
        e.iodc = 1.0;
        e.toe = 417584.0;
        e.toc = 417584.0;
        e.tgd = -1.5366822481155396e-08;
        e.af2 = 0.0;
        e.af1 = 0.0;
        e.af0 = 0.00015433365479111671;
        e.e = 0.024830052279867232;
        e.sqrtA = 5153.651914596558;
        e.deltan = 4.473757778540157e-09;
        e.m0 = 2.5666019628052834;
        e.omega0 = -2.74067902793949;
        e.omega = -0.7949074103300398;
        e.omegaDot = -8.046406593703592e-09;
        e.i0 = 0.9617676328652128;
        e.iDot = -8.107480566251122e-11;
        e.cuc = 3.1329691410064697e-06;
        e.cus = 2.998858690261841e-06;
        e.cic = 3.4086406230926514e-07;
        e.cis = 4.1909515857696533e-07;
        e.crc = 317.9375;
        e.crs = 56.6875;
        e.ura = 0;
        e.health = 0;
        eph(i).SetEphem(e);
        break;
      case 6:
        // i == 6: GPS30
        ToW(i) = 413658.0;
        cno(i) = std::pow(10.0, 46.474539437364385 / 10.0);
        Doppler(i) = 1521.9181867228265;
        CodePhase(i) = 12222.966067242693;

        e.iode = 29.0;
        e.iodc = 29.0;
        e.toe = 417600.0;
        e.toc = 417600.0;
        e.tgd = 3.725290298461914e-09;
        e.af2 = 0.0;
        e.af1 = 1.5916157281026244e-12;
        e.af0 = -0.0005266270600259304;
        e.e = 0.006243427633307874;
        e.sqrtA = 5153.6155433654785;
        e.deltan = 5.39665336370284e-09;
        e.m0 = -0.3340577383885381;
        e.omega0 = 0.4960166323751321;
        e.omega = -2.6350256783905412;
        e.omegaDot = -8.43249410436956e-09;
        e.i0 = 0.9358450024550338;
        e.iDot = 2.2500937254353336e-10;
        e.cuc = 1.0542571544647217e-06;
        e.cus = 6.819143891334534e-06;
        e.cic = 2.0489096641540527e-08;
        e.cis = 9.12696123123169e-08;
        e.crc = 234.25;
        e.crs = 18.125;
        e.ura = 0;
        e.health = 0;
        eph(i).SetEphem(e);
        break;
    }
  }
}

int main() {
  // Create spdlog multi-threaded console/terminal logger
  std::shared_ptr<spdlog::logger> console = spdlog::stdout_color_mt("sturdr-console");
  console->set_pattern("\033[1;34m[%D %T.%e][%^%l%$\033[1;34m]: \033[0m%v");

  double BETA = navtools::LIGHT_SPEED<double> / satutils::GPS_CA_CODE_RATE<>;
  double LAMBDA = navtools::LIGHT_SPEED<double> / satutils::GPS_L1_FREQUENCY<>;
  double T = 0.02;
  Eigen::Vector<satutils::KeplerEphem<double>, 7> eph;
  Eigen::Vector<double, 7> ToW;
  Eigen::Vector<double, 7> CodePhase;
  Eigen::Vector<double, 7> Doppler;
  Eigen::Vector<double, 7> cno;
  gen_nav_data(eph, ToW, CodePhase, Doppler, cno);

  // Transmit time
  Eigen::Vector<double, 7> transmit_times =
      ToW.array() + (CodePhase.array() / satutils::GPS_CA_CODE_RATE<>);
  console->info("transmit_times: [{}]\n", transmit_times.transpose());

  // 1. Calculate satellite positions
  Eigen::Matrix<double, 3, 7> sv_pos = Eigen::Matrix<double, 3, 7>::Zero();
  Eigen::Matrix<double, 3, 7> sv_vel = Eigen::Matrix<double, 3, 7>::Zero();
  Eigen::Matrix<double, 3, 7> sv_acc = Eigen::Matrix<double, 3, 7>::Zero();
  Eigen::Matrix<double, 3, 7> sv_clk = Eigen::Matrix<double, 3, 7>::Zero();
  Eigen::Vector<double, 7> tgd;
  Eigen::Vector<double, 7> psr_var;
  Eigen::Vector<double, 7> psrdot_var;
  for (int i = 0; i < 7; i++) {
    // get satellite positions
    eph(i).CalcNavStates<false>(
        sv_clk.col(i), sv_pos.col(i), sv_vel.col(i), sv_acc.col(i), transmit_times(i));
    tgd(i) = eph(i).tgd;
    psr_var(i) = BETA * BETA * sturdr::DllVariance(cno(i), T);
    psrdot_var(i) = LAMBDA * LAMBDA * sturdr::FllVariance(cno(i), T);
  }
  console->info("sv_pos: [\n{}\n]", sv_pos);
  console->info("sv_vel: [\n{}\n]", sv_vel);
  console->info("sv_clk: [\n{}\n]\n", sv_clk);

  // 2. Estimate pseudoranges and pseudorange-rates
  double receive_time = transmit_times.maxCoeff() + 0.068802;
  Eigen::Vector<double, 7> psr =
      navtools::LIGHT_SPEED<double> *
      (receive_time - transmit_times.array() + sv_clk.row(0).transpose().array() - tgd.array());
  Eigen::Vector<double, 7> psrdot =
      -LAMBDA * Doppler.array() + navtools::LIGHT_SPEED<double> * sv_clk.row(1).transpose().array();

  console->info("psr: [{}]", psr.transpose());
  console->info("psrdot: [{}]\n", psrdot.transpose());

  // 3. Least squares position estimation
  Eigen::Vector<double, 8> x = Eigen::Vector<double, 8>::Zero();
  Eigen::Matrix<double, 8, 8> P = Eigen::Matrix<double, 8, 8>::Zero();
  sturdins::GaussNewton(x, P, sv_pos, sv_vel, psr, psrdot, psr_var, psrdot_var);

  Eigen::Vector3d xyz = x.segment(0, 3);
  Eigen::Vector3d lla = navtools::ecef2lla(xyz).array() * navtools::LLA_RAD2DEG<double>.array();

  console->info("lla: [{}]", lla.transpose());
  console->info("xyz: [{}]", x.segment(0, 3).transpose());
  console->info("xyzv: [{}]", x.segment(3, 3).transpose());
  console->info("cb: {}", x(6));
  console->info("cd: {}", x(7));
  console->info("P: [{}]", P.diagonal().transpose());

  return 0;
}
