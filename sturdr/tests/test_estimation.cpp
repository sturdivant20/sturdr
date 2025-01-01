#include <Eigen/Dense>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <navtools/constants.hpp>
#include <navtools/frames.hpp>

#include "sturdr/nav/ephemeris.hpp"
#include "sturdr/nav/estimation.hpp"
#include "sturdr/utils/gnss-constants.hpp"

void gen_nav_data(
    Eigen::Vector<sturdr::Ephemerides, 7> &e,
    Eigen::Vector<double, 7> &ToW,
    Eigen::Vector<double, 7> &CodePhase,
    Eigen::Vector<double, 7> &Doppler,
    Eigen::Vector<double, 7> &cno) {
  for (int i = 0; i < 7; i++) {
    switch (i) {
      case 0:
        // i == 0: GPS1
        ToW(i) = 413658.0;
        cno(i) = std::pow(10.0, 47.81385205631905 / 10.0);
        Doppler(i) = 1791.6270479363234;
        CodePhase(i) = 11421.606237540786;

        e[i].iode = 86.0;
        e[i].iodc = 86.0;
        e[i].toe = 417600.0;
        e[i].toc = 417600.0;
        e[i].tgd = 4.6566128730773926e-09;
        e[i].af2 = 0.0;
        e[i].af1 = -3.751665644813329e-12;
        e[i].af0 = 0.00020215753465890884;
        e[i].e = 0.012527465354651213;
        e[i].sqrtA = 5153.652446746826;
        e[i].deltan = 3.991594837375442e-09;
        e[i].m0 = 0.4141951882521321;
        e[i].omega0 = -2.6390625759606716;
        e[i].omega = 0.9415036207877718;
        e[i].omegaDot = -8.228914195877791e-09;
        e[i].i0 = 0.9897954243352285;
        e[i].iDot = -3.893019302737323e-11;
        e[i].cuc = 2.0042061805725098e-06;
        e[i].cus = 3.2354146242141724e-06;
        e[i].cic = -7.82310962677002e-08;
        e[i].cis = 1.3224780559539795e-07;
        e[i].crc = 329.78125;
        e[i].crs = 34.625;
        e[i].ura = 0;
        e[i].health = 0;
        break;
      case 1:
        // i == 1: GPS7
        ToW(i) = 413658.0;
        cno(i) = std::pow(10.0, 45.636383357667995 / 10.0);
        Doppler(i) = -473.89562817800544;
        CodePhase(i) = 8209.10940035235;

        e[i].iode = 42.0;
        e[i].iodc = 42.0;
        e[i].toe = 417600.0;
        e[i].toc = 417600.0;
        e[i].tgd = -1.1175870895385742e-08;
        e[i].af2 = 0.0;
        e[i].af1 = -7.73070496506989e-12;
        e[i].af0 = 0.00022176513448357582;
        e[i].e = 0.01688421959988773;
        e[i].sqrtA = 5153.754594802856;
        e[i].deltan = 5.156643366323071e-09;
        e[i].m0 = -0.17767922733872102;
        e[i].omega0 = 0.4888296523228508;
        e[i].omega = -2.214549307999104;
        e[i].omegaDot = -8.492496603714501e-09;
        e[i].i0 = 0.950720208032768;
        e[i].iDot = 2.3393831589843546e-10;
        e[i].cuc = 1.2312084436416626e-06;
        e[i].cus = 7.072463631629944e-06;
        e[i].cic = -5.029141902923584e-08;
        e[i].cis = 2.1979212760925293e-07;
        e[i].crc = 241.53125;
        e[i].crs = 20.53125;
        e[i].ura = 0;
        e[i].health = 0;
        break;
      case 2:
        // i == 1: GPS14
        ToW(i) = 413658.0;
        cno(i) = std::pow(10.0, 45.55573223941419 / 10.0);
        Doppler(i) = 3587.798847894308;
        CodePhase(i) = 11324.089435418902;

        e[i].iode = 201.0;
        e[i].iodc = 457.0;
        e[i].toe = 417600.0;
        e[i].toc = 417600.0;
        e[i].tgd = -7.916241884231567e-09;
        e[i].af2 = 0.0;
        e[i].af1 = 1.0231815394945443e-11;
        e[i].af0 = -2.1620653569698334e-06;
        e[i].e = 0.0027978061698377132;
        e[i].sqrtA = 5153.666526794434;
        e[i].deltan = 4.9441345144764e-09;
        e[i].m0 = -0.9309618696815247;
        e[i].omega0 = 1.5659970309306777;
        e[i].omega = -3.0740762061501883;
        e[i].omegaDot = -8.105694777580142e-09;
        e[i].i0 = 0.949123931804313;
        e[i].iDot = -3.507288949805552e-10;
        e[i].cuc = 1.4360994100570679e-06;
        e[i].cus = 7.890164852142334e-06;
        e[i].cic = 6.146728992462158e-08;
        e[i].cis = -1.862645149230957e-09;
        e[i].crc = 221.46875;
        e[i].crs = 28.15625;
        e[i].ura = 0;
        e[i].health = 0;
        break;
      case 3:
        // i == 3: GPS17
        ToW(i) = 413658.0;
        cno(i) = std::pow(10.0, 46.2857705159281 / 10.0);
        Doppler(i) = 3279.7089666038046;
        CodePhase(i) = 10902.80528025291;

        e[i].iode = 95.0;
        e[i].iodc = 95.0;
        e[i].toe = 417600.0;
        e[i].toc = 417600.0;
        e[i].tgd = -1.1175870895385742e-08;
        e[i].af2 = 0.0;
        e[i].af1 = 2.0463630789890885e-12;
        e[i].af0 = 0.0007058251649141312;
        e[i].e = 0.013569202041253448;
        e[i].sqrtA = 5153.734893798828;
        e[i].deltan = 4.035882396415757e-09;
        e[i].m0 = 2.5834869705907755;
        e[i].omega0 = 2.642788161637698;
        e[i].omega = -1.4142874770588107;
        e[i].omegaDot = -7.60138805689527e-09;
        e[i].i0 = 0.975305087634542;
        e[i].iDot = -2.239378993409451e-10;
        e[i].cuc = 5.0067901611328125e-06;
        e[i].cus = 7.33695924282074e-06;
        e[i].cic = 1.993030309677124e-07;
        e[i].cis = -5.21540641784668e-08;
        e[i].crc = 239.875;
        e[i].crs = 93.84375;
        e[i].ura = 0;
        e[i].health = 0;
        break;
      case 4:
        // i == 4: GPS19
        ToW(i) = 413658.0;
        cno(i) = std::pow(10.0, 43.99677860992774 / 10.0);
        Doppler(i) = 5023.396062348471;
        CodePhase(i) = 3954.6679405210402;

        e[i].iode = 48.0;
        e[i].iodc = 48.0;
        e[i].toe = 417600.0;
        e[i].toc = 417600.0;
        e[i].tgd = -1.5366822481155396e-08;
        e[i].af2 = 0.0;
        e[i].af1 = 5.115907697472721e-12;
        e[i].af0 = 0.00029091350734233856;
        e[i].e = 0.009184657246805727;
        e[i].sqrtA = 5153.615423202515;
        e[i].deltan = 4.096956368963288e-09;
        e[i].m0 = -1.5414014968965324;
        e[i].omega0 = 2.6873690028372486;
        e[i].omega = 2.2537891669538777;
        e[i].omegaDot = -7.92890169915308e-09;
        e[i].i0 = 0.9743497831108455;
        e[i].iDot = -5.928818387655006e-11;
        e[i].cuc = 5.358830094337463e-06;
        e[i].cus = 7.88085162639618e-06;
        e[i].cic = -1.825392246246338e-07;
        e[i].cis = -5.587935447692871e-09;
        e[i].crc = 237.53125;
        e[i].crs = 102.09375;
        e[i].ura = 0;
        e[i].health = 0;
        break;
      case 5:
        // i == 5: GPS21
        ToW(i) = 413658.0;
        cno(i) = std::pow(10.0, 45.96162013805926 / 10.0);
        Doppler(i) = -340.9465647641514;
        CodePhase(i) = 4235.67154014668;

        e[i].iode = 1.0;
        e[i].iodc = 1.0;
        e[i].toe = 417584.0;
        e[i].toc = 417584.0;
        e[i].tgd = -1.5366822481155396e-08;
        e[i].af2 = 0.0;
        e[i].af1 = 0.0;
        e[i].af0 = 0.00015433365479111671;
        e[i].e = 0.024830052279867232;
        e[i].sqrtA = 5153.651914596558;
        e[i].deltan = 4.473757778540157e-09;
        e[i].m0 = 2.5666019628052834;
        e[i].omega0 = -2.74067902793949;
        e[i].omega = -0.7949074103300398;
        e[i].omegaDot = -8.046406593703592e-09;
        e[i].i0 = 0.9617676328652128;
        e[i].iDot = -8.107480566251122e-11;
        e[i].cuc = 3.1329691410064697e-06;
        e[i].cus = 2.998858690261841e-06;
        e[i].cic = 3.4086406230926514e-07;
        e[i].cis = 4.1909515857696533e-07;
        e[i].crc = 317.9375;
        e[i].crs = 56.6875;
        e[i].ura = 0;
        e[i].health = 0;
        break;
      case 6:
        // i == 6: GPS30
        ToW(i) = 413658.0;
        cno(i) = std::pow(10.0, 46.474539437364385 / 10.0);
        Doppler(i) = 1521.9181867228265;
        CodePhase(i) = 12222.966067242693;

        e[i].iode = 29.0;
        e[i].iodc = 29.0;
        e[i].toe = 417600.0;
        e[i].toc = 417600.0;
        e[i].tgd = 3.725290298461914e-09;
        e[i].af2 = 0.0;
        e[i].af1 = 1.5916157281026244e-12;
        e[i].af0 = -0.0005266270600259304;
        e[i].e = 0.006243427633307874;
        e[i].sqrtA = 5153.6155433654785;
        e[i].deltan = 5.39665336370284e-09;
        e[i].m0 = -0.3340577383885381;
        e[i].omega0 = 0.4960166323751321;
        e[i].omega = -2.6350256783905412;
        e[i].omegaDot = -8.43249410436956e-09;
        e[i].i0 = 0.9358450024550338;
        e[i].iDot = 2.2500937254353336e-10;
        e[i].cuc = 1.0542571544647217e-06;
        e[i].cus = 6.819143891334534e-06;
        e[i].cic = 2.0489096641540527e-08;
        e[i].cis = 9.12696123123169e-08;
        e[i].crc = 234.25;
        e[i].crs = 18.125;
        e[i].ura = 0;
        e[i].health = 0;
        break;
    }
  }
}

int main() {
  std::cout << std::setprecision(15);
  double BETA = navtools::LIGHT_SPEED<double> / sturdr::GPS_L1CA_CODE_FREQ;
  double LAMBDA = navtools::LIGHT_SPEED<double> / sturdr::GPS_L1CA_CARRIER_FREQ;
  double T = 0.02;
  Eigen::Vector<sturdr::Ephemerides, 7> eph;
  Eigen::Vector<double, 7> ToW;
  Eigen::Vector<double, 7> CodePhase;
  Eigen::Vector<double, 7> Doppler;
  Eigen::Vector<double, 7> cno;
  gen_nav_data(eph, ToW, CodePhase, Doppler, cno);

  // Transmit time
  Eigen::Vector<double, 7> transmit_times =
      ToW.array() + (CodePhase.array() / sturdr::GPS_L1CA_CODE_FREQ);
  std::cout << "transmit_times: {" << transmit_times.transpose() << "}\n\n";

  // 1. Calculate satellite positions
  Eigen::Matrix<double, 3, 7> sv_pos = Eigen::Matrix<double, 3, 7>::Zero();
  Eigen::Matrix<double, 3, 7> sv_vel = Eigen::Matrix<double, 3, 7>::Zero();
  Eigen::Matrix<double, 3, 7> sv_acc = Eigen::Matrix<double, 3, 7>::Zero();
  Eigen::Matrix<double, 3, 7> sv_clk = Eigen::Matrix<double, 3, 7>::Zero();
  Eigen::Vector<double, 7> tgd;
  for (int i = 0; i < 7; i++) {
    sturdr::GetSvNavStates<false>(
        sv_clk.col(i), sv_pos.col(i), sv_vel.col(i), sv_acc.col(i), transmit_times(i), eph(i));
    tgd(i) = eph(i).tgd;
  }
  std::cout << "sv_pos: {\n" << sv_pos.transpose() << "\n}\n";
  std::cout << "sv_vel: {\n" << sv_vel.transpose() << "\n}\n";
  std::cout << "sv_clk: {\n" << sv_clk.transpose() << "\n}\n\n";

  // 2. Estimate pseudoranges and pseudorange-rates
  double receive_time = transmit_times.maxCoeff() + 0.068802;
  Eigen::Vector<double, 7> psr =
      navtools::LIGHT_SPEED<double> *
      (receive_time - transmit_times.array() + sv_clk.row(0).transpose().array() - tgd.array());
  Eigen::Vector<double, 7> psrdot =
      -LAMBDA * Doppler.array() + navtools::LIGHT_SPEED<double> * sv_clk.row(1).transpose().array();
  std::cout << "psr: {" << psr.transpose() << "}\n";
  std::cout << "psrdot: {" << psrdot.transpose() << "}\n\n";

  // 3. Least squares position estimation
  Eigen::Vector<double, 8> x = Eigen::Vector<double, 8>::Zero();
  Eigen::Matrix<double, 8, 8> P = Eigen::Matrix<double, 8, 8>::Zero();
  sturdr::LeastSquares(x, P, sv_pos, sv_vel, psr, psrdot, cno, BETA, LAMBDA, T);
  Eigen::Vector3d lla =
      navtools::ecef2lla(x.segment(0, 3)).array() * navtools::LLA_RAD2DEG<double>.array();

  std::cout << "lla: {" << lla.transpose() << "}\n";
  std::cout << "xyz: {" << x.segment(0, 3).transpose() << "}\n";
  std::cout << "xyzv: {" << x.segment(3, 3).transpose() << "}\n";
  std::cout << "cb: " << x(6) << "\n";
  std::cout << "cd: " << x(7) << "\n";
  std::cout << "P: {" << P.diagonal().transpose() << "}\n";

  return 0;
}
