"""

GNSS Signal
===========

Common GNSS signal functions.
"""

from __future__ import annotations
import numpy
import typing

__all__ = ["AccumulateEPL", "CarrierNCO", "CircShift", "CodeNCO", "Correlate"]

def AccumulateEPL(
    rfdata: numpy.ndarray[numpy.complex128[m, 1]],
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
    """

def CarrierNCO(
    code: float, code_freq: float, samp_freq: float, rem_code_phase: float, n_samp: int
) -> numpy.ndarray[numpy.complex128[m, 1]]:
    """
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
    """

def CircShift(
    vec: numpy.ndarray[numpy.complex128[m, 1]], shift: int
) -> numpy.ndarray[numpy.complex128[m, 1]]:
    """
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
    """

@typing.overload
def CodeNCO(
    code: bool, code_freq: float, samp_freq: float, rem_code_phase: float
) -> numpy.ndarray[numpy.complex128[m, 1]]:
    """
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
    """

@typing.overload
def CodeNCO(
    code: bool, code_freq: float, samp_freq: float, rem_code_phase: float, n_samp: int
) -> numpy.ndarray[numpy.complex128[m, 1]]:
    """
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
    """

def Correlate(
    rfdata: numpy.ndarray[numpy.complex128[m, 1]],
    carr: numpy.ndarray[numpy.complex128[m, 1]],
    code: numpy.ndarray[numpy.complex128[m, 1]],
) -> complex:
    """
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
    """
