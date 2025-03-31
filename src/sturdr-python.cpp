/**
 * *sturdr-python.cpp*
 *
 * =======  ========================================================================================
 * @file    src/sturdr-python.cpp
 * @brief   PyBind11 wrapper for using SturDR in python!
 * @date    March 2025
 * =======  ========================================================================================
 */

// TODO: add vector processing

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "sturdr/acquisition.hpp"
#include "sturdr/beamformer.hpp"
#include "sturdr/discriminator.hpp"
#include "sturdr/fftw-wrapper.hpp"
#include "sturdr/gnss-signal.hpp"
#include "sturdr/lock-detectors.hpp"
#include "sturdr/sturdr.hpp"
#include "sturdr/tracking.hpp"

namespace py = pybind11;
using namespace sturdr;

PYBIND11_MODULE(_sturdr_core, h) {
  h.doc() = R"pbdoc(
    SturDR
    ======
    
    Daniel Sturdivant's GNSS Software Defined Radio

    Contains the following modules:

    1. `acquisition`
    2. `discriminator`
    3. `gnsssignal`
    4. `lockdetectors`
    5. `tracking`
    6. `BeamFormer`
    7. `FftwWrapper`
    8. `SturDR`
    )pbdoc";
  h.attr("__version__") = "1.0.0";

  // py::class_<fftw_plan>(h, "fftw_plan");
  // py::class_<FftPlans>(h, "FftPlans")
  //     .def_readwrite("fft", &FftPlans::fft)
  //     .def_readwrite("ifft", &FftPlans::ifft)
  //     .def_readwrite("fft_many", &FftPlans::fft_many)
  //     .def_readwrite("ifft_many", &FftPlans::ifft_many);

  //! === Acquisition ==============================================================================
  py::module_ acq = h.def_submodule("acquisition", R"pbdoc(
    Acquisition
    ===========

    Satellite acquisition methods.)pbdoc");

  // PcpsSearch
  acq.def(
      "PcpsSearch",
      &PcpsSearch,
      py::arg("p"),
      py::arg("rfdata"),
      py::arg("code"),
      py::arg("d_range"),
      py::arg("d_step"),
      py::arg("samp_freq"),
      py::arg("code_freq"),
      py::arg("intmd_freq"),
      py::arg("c_per") = 1,
      py::arg("nc_per") = 1,
      R"pbdoc(
      PcpsSearch
      ==========

      Implementation of the parallel code phase search (PCPS) method

      Parameters
      ----------

      p : FftwWrapper

          fftw plans for acquisition

      rfdata : np.ndarray

          Data samples recorded by the RF front end

      code : np.ndarray

          Local code (not upsampled)

      d_range : double

          Max doppler frequency to search [Hz]

      d_step : double

          Frequency step for doppler search [Hz]

      samp_freq : double

          Front end sampling frequency [Hz]

      code_freq : double

          GNSS signal code frequency [Hz]

      intmd_freq : double

          Intermediate frequency of the RF signal [Hz]

      c_per : uint8

          Number of coherent integrations to perform, by default 1

      nc_per : uint8

          Number of non-coherent periods to accumulate, by default 1

      Returns
      -------

      corr_map : np.ndarray

          2D correlation results
      )pbdoc");

  // Peak2NoiseFloorTest
  acq.def(
      "Peak2NoiseFloorTest",
      &Peak2NoiseFloorTest,
      py::arg("corr_map"),
      py::arg("peak_idx"),
      py::arg("metric"),
      R"pbdoc(
      Peak2NoiseFloorTest
      ===================

      Compares the two highest peak to the noise floor of the acquisition plane

      Parameters
      ----------

      corr_map : np.ndarray

          2D-array from correlation method

      peak_idx : np.ndarray

          Indexes of highest correlation peak

      metric : double

          Ratio between the highest peak and noise
      )pbdoc");

  // GlrTest
  acq.def(
      "GlrTest",
      &GlrtTest,
      py::arg("corr_map"),
      py::arg("peak_idx"),
      py::arg("metric"),
      py::arg("rfdata"),
      R"pbdoc(
      GlrTest
      =======

      General likelihood ratio test

      Parameters
      ----------

      corr_map : np.ndarray

          2D-array from correlation method

      peak_idx : np.ndarray

          Indexes of highest correlation peak

      metric : double

          Ratio between the highest peak and noise

      rfdata : np.ndarray

          Data samples recorded by the RF front end
      )pbdoc");

  //! === Beam Steer ===============================================================================
  py::class_<BeamFormer>(h, "BeamFormer")
      .def(
          py::init<int, double, Eigen::Matrix3Xd>(),
          py::arg("n_ant"),
          py::arg("lamb"),
          py::arg("ant_xyz"),
          R"pbdoc(
          BeamFormer
          ==========

          Constructor
          
          Parameters
          ----------

          n_ant : int

              number of antennas

          lamb : double

              wavelength of carrier signal [m/rad]

          ant_xyz : np.ndarray

              3 x n_ant antenna body frame coordinates
          )pbdoc")
      .def("GetWeights", &BeamFormer::GetWeights, R"pbdoc(
          GetWeights
          ==========
          
          Grab the current weights of the beamformer
          
          Returns
          =======

          W : np.ndarray

              current weighting vector
          )pbdoc")
      .def("CalcSteeringWeights", &BeamFormer::CalcSteeringWeights, py::arg("u_body"), R"pbdoc(
          CalcSteeringWeights
          ===================

          Calculates deterministic beam steering weights

          Parameters
          ==========

          u_body : np.ndarray

              unit vector to beam steer towards in the body frame
          )pbdoc")
      .def("CalcNullingWeights", &BeamFormer::CalcNullingWeights, py::arg("u_body"), R"pbdoc(
          CalcNullingWeights
          ==================

          Calculates deterministic null steering weights

          Parameters
          ==========

          u_body : np.ndarray

              unit vector to beam steer towards in the body frame
          )pbdoc")
      .def("__call__", &BeamFormer::operator(), py::arg("x"), R"pbdoc(
          ()
          ==

          The beamforming operation (combines x elements into 1 beamformed element)

          Parameters
          ----------

          x : np.ndarray

              vector of complex elements to beamform together

          Returns
          -------

          y : np.ndarray

              beamformed combination of x
          )pbdoc")
      .doc() = R"pbdoc(
          BeamFormer
          ==========

          Simple, deterministic beamsteering for GNSS phased array antenna.)pbdoc";

  //! === Discriminator ============================================================================
  py::module_ disc = h.def_submodule("discriminator", R"pbdoc(
    Discriminator
    =============

    Standard satellite tracking match filter discriminators.)pbdoc");

  // DllNneml
  disc.def(
      "DllNneml",
      py::overload_cast<const std::complex<double> &, const std::complex<double> &>(&DllNneml),
      py::arg("E"),
      py::arg("L"),
      R"pbdoc(
      DllNneml
      ========

      Delay Lock Loop - Normalized non-coherent early minus late discriminator

      Parameters
      ----------

      E : complex(double)

          Early correlator

      L : complex(double)

          Late correlator

      Returns
      -------

      tau : double

          chip error/misalignment [chip]
      )pbdoc");
  disc.def(
      "DllNneml",
      py::overload_cast<const double &, const double &, const double &, const double &>(&DllNneml),
      py::arg("IE"),
      py::arg("QE"),
      py::arg("IL"),
      py::arg("QL"),
      R"pbdoc(
      DllNneml
      ========

      Delay Lock Loop - Normalized non-coherent early minus late discriminator

      Parameters
      ----------

      IE : complex(double)

          In-phase Early correlator

      QE : complex(double)

          Quadrature Early correlator

      IL : complex(double)

          In-phase Late correlator

      QL : complex(double)

          Quadrature Late correlator

      Returns
      -------

      tau : double

          chip error/misalignment [chip]
      )pbdoc");

  // DllNneml2
  disc.def(
      "DllNneml2",
      py::overload_cast<const std::complex<double> &, const std::complex<double> &>(&DllNneml2),
      py::arg("E"),
      py::arg("L"),
      R"pbdoc(
      DllNneml2
      =========

      Delay Lock Loop - Normalized non-coherent early minus late discriminator squared

      Parameters
      ----------

      E : complex(double)

          Early correlator

      L : complex(double)

          Late correlator

      Returns
      -------

      tau : double

          chip error/misalignment [chip]
      )pbdoc");
  disc.def(
      "DllNneml2",
      py::overload_cast<const double &, const double &, const double &, const double &>(&DllNneml2),
      py::arg("IE"),
      py::arg("QE"),
      py::arg("IL"),
      py::arg("QL"),
      R"pbdoc(
      DllNneml2
      =========

      Delay Lock Loop - Normalized non-coherent early minus late discriminator squared

      Parameters
      ----------

      IE : complex(double)

          In-phase Early correlator

      QE : complex(double)

          Quadrature Early correlator

      IL : complex(double)

          In-phase Late correlator

      QL : complex(double)

          Quadrature Late correlator

      Returns
      -------

      tau : double

          chip error/misalignment [chip]
      )pbdoc");

  // PllCostas
  disc.def(
      "PllCostas",
      py::overload_cast<const std::complex<double> &>(&PllCostas),
      py::arg("P"),
      R"pbdoc(
      PllCostas
      =========

      Phase Lock Loop - Costas discriminator

      Parameters
      ----------

      P : complex(double)

          Prompt correlator

      Returns
      -------

      phi : double

          Phase error/misalignment [rad]
      )pbdoc");
  disc.def(
      "PllCostas",
      py::overload_cast<const double &, const double &>(&PllCostas),
      py::arg("IP"),
      py::arg("QP"),
      R"pbdoc(
      PllCostas
      =========

      Phase Lock Loop - Costas discriminator

      Parameters
      ----------

      IP : complex(double)

          In-phase Prompt correlator

      QP : complex(double)

          Quadrature Prompt correlator

      Returns
      -------

      phi : double

          Phase error/misalignment [rad]
      )pbdoc");

  // PllAtan2
  disc.def(
      "PllAtan2",
      py::overload_cast<const std::complex<double> &>(&PllAtan2),
      py::arg("P"),
      R"pbdoc(
      PllAtan2
      ========

      Phase Lock Loop - ATAN2 discriminator, sensitive to data bits

      Parameters
      ----------

      P : complex(double)

          Prompt correlator

      Returns
      -------

      phi : double

          Phase error/misalignment [rad]
      )pbdoc");
  disc.def(
      "PllAtan2",
      py::overload_cast<const double &, const double &>(&PllAtan2),
      py::arg("IP"),
      py::arg("QP"),
      R"pbdoc(
      PllAtan2
      ========

      Phase Lock Loop - ATAN2 discriminator, sensitive to data bits

      Parameters
      ----------

      IP : complex(double)

          In-phase Prompt correlator

      QP : complex(double)

          Quadrature Prompt correlator

      Returns
      -------

      phi : double

          Phase error/misalignment [rad]
      )pbdoc");

  // FllAtan2
  disc.def(
      "FllAtan2",
      py::overload_cast<const std::complex<double> &, const std::complex<double> &, const double &>(
          &FllAtan2),
      py::arg("P1"),
      py::arg("P2"),
      py::arg("T"),
      R"pbdoc(
      FllAtan2
      ========

      Frequency Lock Loop - ATAN2 discriminator

      Parameters
      ----------

      P1 : complex(double)

          First half prompt correlator

      P2 : complex(double)

          Second half prompt correlator

      T : double

          Integration time of half-correlator [s]

      Returns
      -------

      df : double

          frequency error/misalignment [rad/s]
      )pbdoc");
  disc.def(
      "FllAtan2",
      py::overload_cast<
          const double &,
          const double &,
          const double &,
          const double &,
          const double &>(&FllAtan2),
      py::arg("IP1"),
      py::arg("QP1"),
      py::arg("IP2"),
      py::arg("QP2"),
      py::arg("T"),
      R"pbdoc(
      FllAtan2
      ========

      Frequency Lock Loop - ATAN2 discriminator

      Parameters
      ----------

      IP1 : complex(double)

          In-phase First half prompt correlator

      QP1 : complex(double)

          Quadrature First half prompt correlator

      IP2 : complex(double)

          In-phase Second half prompt correlator

      QP2 : complex(double)

          Quadrature Second half prompt correlator

      T : double

          Integration time of half-correlator [s]

      Returns
      -------

      df : double

          frequency error/misalignment [rad/s]
      )pbdoc");

  // DllVariance
  disc.def(
      "DllVariance",
      &DllVariance,
      py::arg("cno"),
      py::arg("T"),
      R"pbdoc(
      DllVariance
      ===========

      Estimates variance in the DLL discriminator

      Parameters
      ----------

      cno : double

          Carrier to noise density ratio magnitude [Hz]

      T : double

          Integration time [s]

      Returns
      -------

      v : double

          variance in the DLL discriminator [chips^2]
      )pbdoc");

  // PllVariance
  disc.def(
      "PllVariance",
      &PllVariance,
      py::arg("cno"),
      py::arg("T"),
      R"pbdoc(
      PllVariance
      ===========

      Estimates variance in the PLL discriminator

      Parameters
      ----------

      cno : double

          Carrier to noise density ratio magnitude [Hz]

      T : double

          Integration time [s]

      Returns
      -------

      v : double

          variance in the PLL discriminator [rad^2]
      )pbdoc");

  // FllVariance
  disc.def(
      "FllVariance",
      &FllVariance,
      py::arg("cno"),
      py::arg("T"),
      R"pbdoc(
      FllVariance
      ===========

      Estimates variance in the FLL discriminator

      Parameters
      ----------

      cno : double

          Carrier to noise density ratio magnitude [Hz]

      T : double

          Integration time [s]

      Returns
      -------

      v : double

          variance in the FLL discriminator [(rad/s)^2]
      )pbdoc");

  //! === FFTW Wrapper =============================================================================
  py::class_<FftwWrapper>(h, "FftwWrapper")
      .def(py::init<>())
      // .def_readonly("fft_", &FftwWrapper::fft_)
      // .def_readonly("ifft_", &FftwWrapper::ifft_)
      // .def_readonly("fft_many_", &FftwWrapper::fft_many_)
      // .def_readonly("ifft_many_", &FftwWrapper::ifft_many_)
      .def("ThreadSafety", &FftwWrapper::ThreadSafety)
      .def(
          "Create1dFftPlan",
          &FftwWrapper::Create1dFftPlan,
          py::arg("len"),
          py::arg("is_fft"),
          R"pbdoc(
          Create1dFftPlan
          ===============

          Create a complex-to-complex 1d FFT plan

          Parameters
          ----------

          len : int

              length of fft

          is_fft : bool

              boolean to decide whether to create FFT or IFFT plan, defaults to True
          )pbdoc")
      .def(
          "CreateManyFftPlan",
          &FftwWrapper::CreateManyFftPlan,
          py::arg("nrows"),
          py::arg("ncols"),
          py::arg("is_fft"),
          py::arg("is_rowwise"),
          R"pbdoc(
          CreateManyFftPlan
          =================

          Create multiple complex-to-complex 1d FFT plans

          Parameters
          ----------

          nrow : int

              number of rows in matrix

          ncol : int

              number of columns in matrix

          is_fft : bool

              boolean to decide whether to create FFT or IFFT plan, defaults to True

          is_rowwise : bool

              boolean to decide whether fft of rows or columns should be performed, defaults to True
          )pbdoc")
      .def(
          "ExecuteFftPlan",
          &FftwWrapper::ExecuteFftPlan,
          py::arg("input"),
          py::arg("output"),
          py::arg("is_fft"),
          py::arg("is_many_fft"),
          R"pbdoc(
          ExecuteFftPlan
          ==============

          Perform a complex-to-complex 1d FFT/IFFT

          Parameters
          ----------

          input : np.ndarray

              input data stream

          output : np.ndarray

              output data stream

          is_fft : bool

              boolean to decide whether to use FFT or IFFT plan, defaults to True

          is_many_fft : bool

              boolean to decide whether to 1d or 2d plans, defaults to False

          Returns
          -------

          status : bool

              True|False based on success
          )pbdoc")
      .doc() = R"pbdoc(
          FftwWrapper

          Small functions for using complex-1d FFTW3 methods specific to SturDR.)pbdoc";

  //! === GNSS Signal ==============================================================================
  py::module_ gnss = h.def_submodule("gnsssignal", R"pbdoc(
    GNSS Signal
    ===========

    Common GNSS signal functions.)pbdoc");

  // CircShift
  gnss.def(
      "CircShift",
      &CircShift,
      py::arg("vec"),
      py::arg("shift"),
      R"pbdoc(
      CircShift
      =========

      Circularly shift a vector

      Parameters
      ----------

      vec : np.ndarray

          Vector to shift

      shift : int

          Amount to rotate

      Returns
      -------

      new_vec : np.ndarray

          Shifted vector
      )pbdoc");

  // AccumulateEPL
  gnss.def(
      "AccumulateEPL",
      &AccumulateEPL,
      py::arg("rfdata"),
      py::arg("code"),
      py::arg("rem_code_phase"),
      py::arg("code_freq"),
      py::arg("rem_carr_phase"),
      py::arg("carr_freq"),
      py::arg("carr_jit"),
      py::arg("samp_freq"),
      py::arg("half_samp"),
      py::arg("samp_remaining"),
      py::arg("t_space"),
      py::arg("E"),
      py::arg("P1"),
      py::arg("P2"),
      py::arg("L"),
      R"pbdoc(
      AccumulateEPL
      =============

      Accumulates 'n_samp' samples of the current integration period

      Parameters
      ----------

      rfdata : np.ndarray
      
          Data samples recorded by the RF front end

      code : np.ndarray

          Local code (not upsampled)

      rem_code_phase : double

          Initial fractional phase of the code

      code_freq : double

          GNSS signal code frequency [Hz]

      rem_carr_phase : double

          Initial fractional phase of the carrier [rad]

      carr_freq : double

          Current carrier frequency (including intermediate frequency) [rad/s]

      carr_jit : double

          Current carrier frequency jitter [rad/s^2]

      samp_freq : double

          Front end sampling frequency [Hz]

      half_samp : uint64

          Number of samples in half the TOTAL accumulation period

      samp_remaining : uint64

          Number of samples remaining to be accumulated inside TOTAL period

      t_space : double

          Spacing between correlator taps

      E : complex(double)

          Early correlator

      P1 : complex(double)

          Prompt first-half correlator

      P2 : complex(double)

          Prompt second-half correlator

      L : complex(double)

          Late correlator
      )pbdoc");

  // Correlate
  gnss.def(
      "Correlate",
      py::overload_cast<
          const Eigen::Ref<const Eigen::VectorXcd> &,
          const Eigen::Ref<const Eigen::VectorXcd> &,
          const Eigen::Ref<const Eigen::VectorXcd> &>(&Correlate),
      py::arg("rfdata"),
      py::arg("carr"),
      py::arg("code"),
      R"pbdoc(
      Correlate
      =========

      Correlate a signal to a carrier and code replica

      Parameters
      ----------

      rfdata : np.ndarray
      
          Data samples recorded by the RF front end

      carr : np.ndarray

          Local carrier replica from NCO

      code : np.ndarray

          Local code replica from NCO

      Returns
      -------

      R : complex(double)

          GNSS correlator value
      )pbdoc");

  // CodeNCO
  gnss.def(
      "CodeNCO",
      py::overload_cast<const bool[1023], const double &, const double &, double &>(&CodeNCO),
      py::arg("code"),
      py::arg("code_freq"),
      py::arg("samp_freq"),
      py::arg("rem_code_phase"),
      R"pbdoc(
      CodeNCO
      =======

      Creates an upsampled version of the code provided

      Parameters
      ----------
      
      code : np.ndarray

          Local code (not upsampled)

      code_freq : double

          GNSS signal code frequency [Hz]

      samp_freq : double

          Front end sampling frequency [Hz]

      rem_code_phase : double

          Initial fractional phase of the code

      Returns
      -------

      code_up : np.ndarray

          upsampled code
      )pbdoc");

  // CodeNCO
  gnss.def(
      "CodeNCO",
      py::overload_cast<const bool[1023], const double &, const double &, double &, uint64_t &>(
          &CodeNCO),
      py::arg("code"),
      py::arg("code_freq"),
      py::arg("samp_freq"),
      py::arg("rem_code_phase"),
      py::arg("n_samp"),
      R"pbdoc(
      CodeNCO
      =======

      Creates an upsampled version of the code provided

      Parameters
      ----------
      
      code : np.ndarray

          Local code (not upsampled)

      code_freq : double

          GNSS signal code frequency [Hz]

      samp_freq : double

          Front end sampling frequency [Hz]

      rem_code_phase : double

          Initial fractional phase of the code

      n_samp : uint64

          Length of upsampled code

      Returns
      -------

      code_up : np.ndarray

          upsampled code
      )pbdoc");

  // CarrierNCO
  gnss.def(
      "CarrierNCO",
      &CarrierNCO,
      py::arg("code"),
      py::arg("code_freq"),
      py::arg("samp_freq"),
      py::arg("rem_code_phase"),
      py::arg("n_samp"),
      R"pbdoc(
      CarrierNCO
      ==========

      Creates an sampled version of the carrier wave

      Parameters
      ----------

      carr_freq : double

          Current carrier frequency (including intermediate frequency) [rad/s]

      carr_jit : double

          Current carrier frequency jitter [rad/s^2]

      samp_freq : double

          Front end sampling frequency [Hz]

      rem_carr_phase : double

          Initial fractional phase of the carrier [rad]

      n_samp : uint64

          Length of sampled carrier

      Returns
      -------

      code_up : np.ndarray

          Phase sampled carrier
      )pbdoc");

  //! === Lock Detectors ===========================================================================
  py::module_ lock = h.def_submodule("lockdetectors", R"pbdoc(
    Lock Detectors
    ==============

    Satellite code and carrier lock detectors.)pbdoc");

  // CodeLockDetector
  lock.def(
      "CodeLockDetector",
      py::overload_cast<
          bool &,
          double &,
          double &,
          double &,
          const std::complex<double> &,
          const std::complex<double> &,
          const double &,
          const double &>(&CodeLockDetector),
      py::arg("code_lock"),
      py::arg("cno"),
      py::arg("pc"),
      py::arg("pn"),
      py::arg("P_old"),
      py::arg("P_new"),
      py::arg("T"),
      py::arg("alpha") = 0.005,
      R"pbdoc(
      CodeLockDetector
      ================

      Code lock detector and Carrier-to-Noise ratio estimator (must be reset if integration period changes)

      Parameters
      ----------

      code_lock : bool

          current code lock status

      cno : double

          current carrier-to-noise density ratio [Hz]

      pc : double

          Current estimate of carrier power

      pn : double

          Current estimate of noise power

      P_old : complex(double)

          Old prompt correlator

      P_new : complex(double)

          New prompt correlator

      T : double

          Integtration time/period [s]

      alpha : double

          Smoothing (filtering) coefficient, by default 5e-3
      )pbdoc");

  lock.def(
      "CodeLockDetector",
      py::overload_cast<
          bool &,
          double &,
          double &,
          double &,
          const double &,
          const double &,
          const double &,
          const double &,
          const double &,
          const double &>(&CodeLockDetector),
      py::arg("code_lock"),
      py::arg("cno"),
      py::arg("pc"),
      py::arg("pn"),
      py::arg("IP_old"),
      py::arg("QP_old"),
      py::arg("IP_new"),
      py::arg("QP_new"),
      py::arg("T"),
      py::arg("alpha") = 0.005,
      R"pbdoc(
      CodeLockDetector
      ================

      Code lock detector and Carrier-to-Noise ratio estimator (must be reset if integration period changes)

      Parameters
      ----------

      code_lock : bool

          current code lock status

      cno : double

          current carrier-to-noise density ratio [Hz]

      pc : double

          Current estimate of carrier power

      pn : double

          Current estimate of noise power

      IP_old : complex(double)

          Old in-phase prompt correlator

      QP_old : complex(double)

          Old quadrature prompt correlator

      IP_new : complex(double)

          New in-phase prompt correlator

      QP_new : complex(double)

          New quadrature prompt correlator

      T : double

          Integtration time/period [s]

      alpha : double

          Smoothing (filtering) coefficient, by default 5e-3
      )pbdoc");

  // CarrierLockDetector
  lock.def(
      "CarrierLockDetector",
      py::overload_cast<bool &, double &, double &, const std::complex<double> &, const double &>(
          &CarrierLockDetector),
      py::arg("carr_lock"),
      py::arg("nbd"),
      py::arg("nbp"),
      py::arg("P"),
      py::arg("alpha") = 0.005,
      R"pbdoc(
      CarrierLockDetector
      ===================

      Carrier phase lock detector

      Parameters
      ----------

      carr_lock : bool

          Current carrier lock status

      nbd : double

          Narrow band difference memory

      nbp : double

          Narrow band power memory

      P : complex(double)

          New prompt correlator

      alpha : double

          Smoothing (filtering) coefficient, by default 5e-3
      )pbdoc");

  lock.def(
      "CarrierLockDetector",
      py::overload_cast<bool &, double &, double &, const double &, const double &, const double &>(
          &CarrierLockDetector),
      py::arg("carr_lock"),
      py::arg("nbd"),
      py::arg("nbp"),
      py::arg("IP"),
      py::arg("QP"),
      py::arg("alpha") = 0.005,
      R"pbdoc(
      CarrierLockDetector
      ===================

      Carrier phase lock detector

      Parameters
      ----------

      carr_lock : bool

          Current carrier lock status

      nbd : double

          Narrow band difference memory

      nbp : double

          Narrow band power memory

      IP : complex(double)

          New in-phase prompt correlator

      QP : complex(double)

          New quadrature prompt correlator

      alpha : double

          Smoothing (filtering) coefficient, by default 5e-3
      )pbdoc");

  // LockDetectors
  py::class_<LockDetectors>(lock, "LockDetectors")
      .def(py::init<double>())
      .def(
          "Update",
          py::overload_cast<const double &, const double &, const double &>(&LockDetectors::Update),
          py::arg("IP"),
          py::arg("QP"),
          py::arg("T"))
      .def(
          "Update",
          py::overload_cast<std::complex<double> &, const double &>(&LockDetectors::Update),
          py::arg("P"),
          py::arg("T"))
      .def("GetCodeLock", &LockDetectors::GetCodeLock)
      .def("GetCarrierLock", &LockDetectors::GetCarrierLock)
      .def("GetCno", &LockDetectors::GetCno);

  //! === Tracking =================================================================================
  py::module_ trk = h.def_submodule("tracking", R"pbdoc(
    Tracking
    ========

    Satellite scalar tracking methods.)pbdoc");

  // NaturalFrequency
  trk.def(
      "NaturalFrequency",
      &NaturalFrequency,
      py::arg("bw"),
      py::arg("order"),
      R"pbdoc(
      NaturalFrequency
      ================

      Calculate the natural radian frequency of the loop filter given the noise bandwidth

      Parameters
      ----------

      bw : double

          Noise bandwidth [Hz]

      order : int

          Loop filter order (1, 2, or 3)

      Returns
      -------

      w0 : double

          Loop filter's natural radian frequency [rad/s]
      )pbdoc");

  // FLLassistedPLL_2ndOrder
  trk.def(
      "FLLassistedPLL_2ndOrder",
      &FLLassistedPLL_2ndOrder,
      py::arg("nco_freq"),
      py::arg("vel_accum"),
      py::arg("phase_err"),
      py::arg("freq_err"),
      py::arg("T"),
      py::arg("w0p"),
      py::arg("w0f"),
      R"pbdoc(
      FLLassistedPLL_2ndOrder
      =======================

      2nd order PLL/DLL assisted by 1st order FLL Digital Loop Filter

      Parameters
      ----------

      nco_freq : double

          Doppler/Frequency estimate from the Digital Loop Filter

      vel_accum : double

          Velocity accumulator memory

      phase_err : double

          Phase error input (discriminator) [rad]

      freq_err : double

          Frequency error input (discriminator) [rad/s]

      T : double

          Coherent integration time [s]
          
      w0p : double

          Natural radian frequency of the PLL [rad/s]

      w0f : double

          Natural radian frequency of the FLL [rad/s]
      )pbdoc");

  // FLLassistedPLL_3rdOrder
  trk.def(
      "FLLassistedPLL_3rdOrder",
      &FLLassistedPLL_3rdOrder,
      py::arg("nco_freq"),
      py::arg("vel_accum"),
      py::arg("acc_accum"),
      py::arg("phase_err"),
      py::arg("freq_err"),
      py::arg("T"),
      py::arg("w0p"),
      py::arg("w0f"),
      R"pbdoc(
      FLLassistedPLL_3rdOrder
      =======================

      3rd order PLL/DLL assisted by 2nd order FLL Digital Loop Filter

      Parameters
      ----------

      nco_freq : double

          Doppler/Frequency estimate from the Digital Loop Filter

      vel_accum : double

          Velocity accumulator memory

      acc_accum : double

          Acceleration accumulator memory

      phase_err : double

          Phase error input (discriminator) [rad]

      freq_err : double

          Frequency error input (discriminator) [rad/s]

      T : double

          Coherent integration time [s]
          
      w0p : double

          Natural radian frequency of the PLL [rad/s]

      w0f : double

          Natural radian frequency of the FLL [rad/s]
      )pbdoc");

  // PLL_2ndOrder
  trk.def(
      "PLL_2ndOrder",
      &PLL_2ndOrder,
      py::arg("nco_freq"),
      py::arg("vel_accum"),
      py::arg("phase_err"),
      py::arg("T"),
      py::arg("w0p"),
      R"pbdoc(
      PLL_2ndOrder
      ============

      2nd order PLL/DLL

      Parameters
      ----------

      nco_freq : double

          Doppler/Frequency estimate from the Digital Loop Filter

      vel_accum : double

          Velocity accumulator memory

      phase_err : double

          Phase error input (discriminator) [rad]

      T : double

          Coherent integration time [s]
          
      w0p : double

          Natural radian frequency of the PLL [rad/s]
      )pbdoc");

  // PLL_3rdOrder
  trk.def(
      "PLL_3rdOrder",
      &PLL_3rdOrder,
      py::arg("nco_freq"),
      py::arg("vel_accum"),
      py::arg("acc_accum"),
      py::arg("phase_err"),
      py::arg("T"),
      py::arg("w0p"),
      R"pbdoc(
      PLL_3rdOrder
      =======================

      3rd order PLL/DLL assisted by 2nd order FLL Digital Loop Filter

      Parameters
      ----------

      nco_freq : double

          Doppler/Frequency estimate from the Digital Loop Filter

      vel_accum : double

          Velocity accumulator memory

      acc_accum : double

          Acceleration accumulator memory

      phase_err : double

          Phase error input (discriminator) [rad]

      T : double

          Coherent integration time [s]
          
      w0p : double

          Natural radian frequency of the PLL [rad/s]
      )pbdoc");

  py::class_<TrackingKF>(trk, "TrackingKF")
      .def(py::init<>())
      .def_readwrite("x_", &TrackingKF::x_)
      .def(
          "Init",
          &TrackingKF::Init,
          py::arg("init_carr_phase"),
          py::arg("init_carr_doppler"),
          py::arg("init_code_phase"),
          py::arg("intmd_freq"),
          py::arg("code_freq"),
          R"pbdoc(
          TrackingKF
          ==========
          
          Init function

          Parameters
          ----------

          init_carr_phase : double

              Initial carrier phase estimate [rad]

          init_carr_doppler : double

              Initial doppler estimate [rad/s]

          init_code_phase : double

              Initial code phase estimate [chips]

          intmd_freq : double

              Intermediate frequency of the recorded signal [rad/s]

          code_freq : double
          
              Chipping rate of the true signal [chip/s]
          )pbdoc")
      .def(
          "UpdateDynamicsParam",
          &TrackingKF::UpdateDynamicsParam,
          py::arg("w0d"),
          py::arg("w0p"),
          py::arg("w0f"),
          py::arg("kappa"),
          py::arg("T"),
          R"pbdoc(
          UpdateDynamicsParam
          ===================

          Change the dynamics update parameters

          Parameters
          ----------

          w0d : double

              Natural radian frequency of the DLL [rad/s]

          w0p : double

              Natural radian frequency of the PLL [rad/s]

          w0f : double 

              Natural radian frequency of the FLL [rad/s]

          kappa : double

              Code to carrier frequency ratio [rad/chip]

          T : double

              Integration time [s]
          )pbdoc")
      .def(
          "UpdateMeasurementsParam",
          &TrackingKF::UpdateMeasurementsParam,
          py::arg("dll_var"),
          py::arg("pll_var"),
          py::arg("fll_var"),
          R"pbdoc(
          UpdateMeasurementsParam
          =======================

          Change the measurement update parameters

          Parameters
          ----------

          dll_var : double

              DLL discriminator variance [chip^2]

          pll_var : double

              PLL discriminator variance [rad^2]

          fll_var : double

              FLL discriminator variance [(rad/s)^2]
          )pbdoc")
      .def(
          "Run",
          &TrackingKF::Run,
          py::arg("chip_err"),
          py::arg("phase_err"),
          py::arg("freq_err"),
          R"pbdoc(
          Run
          ===

          Run the Kalman Filter DLL/PLL

          Parameters
          ----------

          chip_err : double

              Chip discriminator [chips]

          phase_err : double

              Phase discriminator [rad]

          freq_err : double

              Frequency discriminator [rad/s]
          )pbdoc")
      .def(
          "SetRemCarrierPhase",
          &TrackingKF::SetRemCarrierPhase,
          py::arg("p"),
          R"pbdoc(
            SetRemCarrierPhase
            ==================
  
            Set remainder carrier phase x[0]
  
            Parameters
            ----------
  
            p : double
  
                carrier phase [rad]
            )pbdoc")
      .def(
          "SetRemCodePhase",
          &TrackingKF::SetRemCodePhase,
          py::arg("c"),
          R"pbdoc(
          SetRemCodePhase
          ===============

          Set remainder code phase x[3]

          Parameters
          ----------

          c : double

              code phase [chip]
          )pbdoc")
      .def(
          "GetRemCarrierPhase",
          &TrackingKF::GetRemCarrierPhase,
          R"pbdoc(
          GetRemCarrierPhase
          ==================

          Get remainder carrier phase x[0]

          Returns
          -------

          p : double

              carrier phase [rad]
          )pbdoc")
      .def(
          "GetRemCodePhase",
          &TrackingKF::GetRemCodePhase,
          R"pbdoc(
          GetRemCodePhase
          ===============

          Get remainder code phase x[3]

          Returns
          -------

          c : double

              code phase [chip]
          )pbdoc")
      .def(
          "GetDoppler",
          &TrackingKF::GetDoppler,
          R"pbdoc(
          GetDoppler
          ===============

          Get carrier doppler x[1]

          Returns
          -------

          f : double

              doppler [rad/s]
          )pbdoc")
      .doc() = R"pbdoc(
          TrackingKF
          ==========

          GPS, scalar tracking, E-P-L, Kalman Filter.
          )pbdoc";

  //! === SturDR ===================================================================================
  py::class_<SturDR>(h, "SturDR")
      .def(py::init<std::string>())
      .def(
          "Start",
          &SturDR::Start,
          R"pbdoc(
          Start
          =====
          
          Initializes and begins running the receiver
          )pbdoc")
      .doc() = R"pbdoc(
          SturDR
          ======

          SturDR receiver implementation.
          )pbdoc";
}