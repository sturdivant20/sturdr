"""

Beam Steer
==========

Simple, deterministic beamsteering for GNSS phased array antenna.
"""

from __future__ import annotations
import numpy

__all__ = ["DeterministicBeam", "DeterministicNull"]

def DeterministicBeam(
    ant_xyz: numpy.ndarray[numpy.float64[m, n], numpy.ndarray.flags.f_contiguous],
    u: numpy.ndarray[numpy.float64[3, 1]],
    rfdata: numpy.ndarray[numpy.complex128[m, n], numpy.ndarray.flags.f_contiguous],
    code: bool,
    rem_code_phase: float,
    code_freq: float,
    rem_carr_phase: float,
    carr_freq: float,
    carr_jit: float,
    samp_freq: float,
    half_samp: int,
    samp_remaining: int,
    t_space: float,
    E: complex,
    P1: complex,
    P2: complex,
    L: complex,
) -> None:
    """
    DeterministicBeam
    =================

    Accumulates 'n_samp' samples of the current integration period and beamsteers toward provided unit vector.

    Parameters
    ----------

    axt_xyz : np.ndarray

        Known antenna positions in the body frame [in cycles (2*pi/lambda*xyz_m)]

    u : np.ndarray

        Desired unit vector in the body frame

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
    """

def DeterministicNull(
    ant_xyz: numpy.ndarray[numpy.float64[m, n], numpy.ndarray.flags.f_contiguous],
    u: numpy.ndarray[numpy.float64[3, 1]],
    rfdata: numpy.ndarray[numpy.complex128[m, n], numpy.ndarray.flags.f_contiguous],
    code: bool,
    rem_code_phase: float,
    code_freq: float,
    rem_carr_phase: float,
    carr_freq: float,
    carr_jit: float,
    samp_freq: float,
    half_samp: int,
    samp_remaining: int,
    t_space: float,
    E: complex,
    P1: complex,
    P2: complex,
    L: complex,
) -> None:
    """
    DeterministicNull
    =================

    Accumulates 'n_samp' samples of the current integration period and nullsteers toward provided unit vector.

    Parameters
    ----------

    axt_xyz : np.ndarray

        Known antenna positions in the body frame [in cycles (2*pi/lambda*xyz_m)]

    u : np.ndarray

        Desired unit vector in the body frame

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
    """
