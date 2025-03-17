"""

SturDR
======

Daniel Sturdivant's GNSS Software Defined Radio

Contains the following modules:

1. `acquisition`
2. `beamsteer`
3. `discriminator`
4. `gnsssignal`
5. `lockdetectors`
6. `tracking`
7. `FftwWrapper`
8. `SturDR`

"""

from __future__ import annotations
import numpy
from . import acquisition
from . import beamsteer
from . import discriminator
from . import gnsssignal
from . import lockdetectors
from . import tracking

__all__ = [
    "FftwWrapper",
    "SturDR",
    "acquisition",
    "beamsteer",
    "discriminator",
    "gnsssignal",
    "lockdetectors",
    "tracking",
]

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
