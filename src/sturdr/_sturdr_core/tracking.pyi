"""

Tracking
========

Satellite scalar tracking methods.
"""

from __future__ import annotations
import numpy

__all__ = [
    "FLLassistedPLL_2ndOrder",
    "FLLassistedPLL_3rdOrder",
    "NaturalFrequency",
    "PLL_2ndOrder",
    "PLL_3rdOrder",
    "TrackingKF",
]

class TrackingKF:
    """

    TrackingKF
    ==========

    GPS, scalar tracking, E-P-L, Kalman Filter.

    """

    x_: numpy.ndarray[numpy.float64[5, 1]]
    def GetDoppler(self) -> float:
        """
        GetDoppler
        ===============

        Get carrier doppler x[1]

        Returns
        -------

        f : double

            doppler [rad/s]
        """

    def GetRemCarrierPhase(self) -> float:
        """
        GetRemCarrierPhase
        ==================

        Get remainder carrier phase x[0]

        Returns
        -------

        p : double

            carrier phase [rad]
        """

    def GetRemCodePhase(self) -> float:
        """
        GetRemCodePhase
        ===============

        Get remainder code phase x[3]

        Returns
        -------

        c : double

            code phase [chip]
        """

    def Init(
        self,
        init_carr_phase: float,
        init_carr_doppler: float,
        init_code_phase: float,
        intmd_freq: float,
        code_freq: float,
    ) -> None:
        """
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
        """

    def Run(self, chip_err: float, phase_err: float, freq_err: float) -> None:
        """
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
        """

    def SetRemCarrierPhase(self, p: float) -> None:
        """
        SetRemCarrierPhase
        ==================

        Set remainder carrier phase x[0]

        Parameters
        ----------

        p : double

            carrier phase [rad]
        """

    def SetRemCodePhase(self, c: float) -> None:
        """
        SetRemCodePhase
        ===============

        Set remainder code phase x[3]

        Parameters
        ----------

        c : double

            code phase [chip]
        """

    def UpdateDynamicsParam(
        self, w0d: float, w0p: float, w0f: float, kappa: float, T: float
    ) -> None:
        """
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
        """

    def UpdateMeasurementsParam(self, dll_var: float, pll_var: float, fll_var: float) -> None:
        """
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
        """

    def __init__(self) -> None: ...

def FLLassistedPLL_2ndOrder(
    nco_freq: float,
    vel_accum: float,
    phase_err: float,
    freq_err: float,
    T: float,
    w0p: float,
    w0f: float,
) -> None:
    """
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
    """

def FLLassistedPLL_3rdOrder(
    nco_freq: float,
    vel_accum: float,
    acc_accum: float,
    phase_err: float,
    freq_err: float,
    T: float,
    w0p: float,
    w0f: float,
) -> None:
    """
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
    """

def NaturalFrequency(bw: float, order: int) -> float:
    """
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
    """

def PLL_2ndOrder(nco_freq: float, vel_accum: float, phase_err: float, T: float, w0p: float) -> None:
    """
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
    """

def PLL_3rdOrder(
    nco_freq: float, vel_accum: float, acc_accum: float, phase_err: float, T: float, w0p: float
) -> None:
    """
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
    """
