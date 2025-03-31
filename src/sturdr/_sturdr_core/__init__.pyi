"""

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

"""

from __future__ import annotations
import numpy
from . import acquisition
from . import discriminator
from . import gnsssignal
from . import lockdetectors
from . import tracking

__all__ = [
    "BeamFormer",
    "FftwWrapper",
    "SturDR",
    "acquisition",
    "discriminator",
    "gnsssignal",
    "lockdetectors",
    "tracking",
]

class BeamFormer:
    """

    BeamFormer
    ==========

    Simple, deterministic beamsteering for GNSS phased array antenna.
    """

    def CalcNullingWeights(self, u_body: numpy.ndarray[numpy.float64[3, 1]]) -> None:
        """
        CalcNullingWeights
        ==================

        Calculates deterministic null steering weights

        Parameters
        ==========

        u_body : np.ndarray

            unit vector to beam steer towards in the body frame
        """

    def CalcSteeringWeights(self, u_body: numpy.ndarray[numpy.float64[3, 1]]) -> None:
        """
        CalcSteeringWeights
        ===================

        Calculates deterministic beam steering weights

        Parameters
        ==========

        u_body : np.ndarray

            unit vector to beam steer towards in the body frame
        """

    def GetWeights(self) -> numpy.ndarray[numpy.complex128[m, 1]]:
        """
        GetWeights
        ==========

        Grab the current weights of the beamformer

        Returns
        =======

        W : np.ndarray

            current weighting vector
        """

    def __call__(self, x: numpy.ndarray[numpy.complex128[m, 1]]) -> complex:
        """
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
        """

    def __init__(
        self, n_ant: int, lamb: float, ant_xyz: numpy.ndarray[numpy.float64[3, n]]
    ) -> None:
        """
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
        """

class FftwWrapper:
    """

    FftwWrapper

    Small functions for using complex-1d FFTW3 methods specific to SturDR.
    """

    def Create1dFftPlan(self, len: int, is_fft: bool) -> None:
        """
        Create1dFftPlan
        ===============

        Create a complex-to-complex 1d FFT plan

        Parameters
        ----------

        len : int

            length of fft

        is_fft : bool

            boolean to decide whether to create FFT or IFFT plan, defaults to True
        """

    def CreateManyFftPlan(self, nrows: int, ncols: int, is_fft: bool, is_rowwise: bool) -> None:
        """
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
        """

    def ExecuteFftPlan(
        self,
        input: numpy.ndarray[
            numpy.complex128[m, n], numpy.ndarray.flags.writeable, numpy.ndarray.flags.f_contiguous
        ],
        output: numpy.ndarray[
            numpy.complex128[m, n], numpy.ndarray.flags.writeable, numpy.ndarray.flags.f_contiguous
        ],
        is_fft: bool,
        is_many_fft: bool,
    ) -> bool:
        """
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
        """

    def ThreadSafety(self) -> None: ...
    def __init__(self) -> None: ...

class SturDR:
    """

    SturDR
    ======

    SturDR receiver implementation.

    """

    def Start(self) -> None:
        """
        Start
        =====

        Initializes and begins running the receiver
        """

    def __init__(self, arg0: str) -> None: ...

__version__: str = "1.0.0"
