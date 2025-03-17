"""

Acquisition
===========

Satellite acquisition methods.
"""

from __future__ import annotations
import numpy

__all__ = ["GlrTest", "PcpsSearch", "Peak2NoiseFloorTest"]

def GlrTest(
    corr_map: numpy.ndarray[numpy.float64[m, n]],
    peak_idx: int,
    metric: float,
    rfdata: numpy.ndarray[numpy.complex128[m, 1]],
) -> None:
    """
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
    """

def PcpsSearch(
    p: ...,
    rfdata: numpy.ndarray[numpy.complex128[m, 1]],
    code: bool,
    d_range: float,
    d_step: float,
    samp_freq: float,
    code_freq: float,
    intmd_freq: float,
    c_per: int = 1,
    nc_per: int = 1,
) -> numpy.ndarray[numpy.float64[m, n]]:
    """
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
    """

def Peak2NoiseFloorTest(
    corr_map: numpy.ndarray[numpy.float64[m, n]], peak_idx: int, metric: float
) -> None:
    """
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
    """
