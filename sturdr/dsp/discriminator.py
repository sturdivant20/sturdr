"""**discriminator.py**

======  ============================================================================================
file    sturdr/dsp/discriminator.py
brief   Standard satellite tracking match filter discriminators.  
date    October 2024
refs    1. "Understanding GPS/GNSS Principles and Applications", 3rd Edition, 2017 
            - Kaplan & Hegarty
        2. "Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems", 2nd Edition, 2013
            - Groves
        3. "Global Positioning System: Signals, Measurements, and Performance", 2nd Edition, 2006
            - Misra & Enge
        4. "Position, Navigation, and Timing Technologies in the 21st Century", Volume 1, 2021
            - Morton, Diggelen, Spilker Jr., Parkinson
======  ============================================================================================
"""

import numpy as np
from numba import njit

# ===== DLL ====================================================================================== #

@njit(cache=True, fastmath=True)
def DllNneml(IE: np.double, QE: np.double, IL: np.double, QL: np.double):
    """
    Delay Lock Loop - Normalized non-coherent early minus late discriminator

    Parameters
    ----------
    IE : np.double
        in-phase early discriminator
    QE : np.double
        quadra-phase early discriminator
    IL : np.double
        in-phase late discriminator
    QL : np.double
        quadra-phase late discriminator

    Returns
    -------
    tau : np.double
        chip error/misalignment [chip]
    """
    E = np.sqrt(IE**2 + QE**2)
    L = np.sqrt(IL**2 + QL**2)
    return 0.5 * (E - L) / (E + L)

@njit(cache=True, fastmath=True)
def DllNcdp(IE: np.double, IP: np.double, IL: np.double):
    """
    Delay Lock Loop - Normalized coherent dot product discriminator

    Parameters
    ----------
    IE : np.double
        in-phase early discriminator
    IP : np.double
        in-phase prompt discriminator
    IL : np.double
        in-phase late discriminator

    Returns
    -------
    tau : np.double
        chip error/misalignment [chip]
    """
    # return 0.25 * (IE - IL) / IP
    return (np.abs(IE) - np.abs(IL)) / np.abs(IP)

@njit(cache=True, fastmath=True)
def DllVariance(cn0: np.double, T: np.double):
    """
    Variance in the DLL discriminator

    Parameters
    ----------
    cn0 : np.double
        Carrier to noise density ratio magnitude (not dB-Hz)
    T : np.double
        Integtation time [s]

    Returns
    -------
    np.double
        variance in the DLL discriminator [chips^2]
    """
    tmp = 1.0 / (cn0 * T)
    return tmp * (0.25 + 0.5 * tmp)

# ===== PLL ====================================================================================== #

@njit(cache=True, fastmath=True)
def PllCostas(IP: np.double, QP: np.double):
    """
    Phase Lock Loop - Costas discriminator

    Parameters
    ----------
    IP : np.double
        in-phase prompt discriminator
    QP : np.double
        quadra-phase prompt discriminator

    Returns
    -------
    phi
        phase error/misalignment [rad]
    """
    return np.atan(QP / IP)

@njit(cache=True, fastmath=True)
def PllAtan2(IP: np.double, QP: np.double):
    """
    Phase Lock Loop - ATAN2 discriminator, sensitive to data bits

    Parameters
    ----------
    IP : np.double
        in-phase prompt discriminator
    QP : np.double
        quadra-phase prompt discriminator

    Returns
    -------
    phi
        phase error/misalignment [rad]
    """
    return np.atan2(QP, IP)

@njit(cache=True, fastmath=True)
def PllNddc(IP: np.double, QP: np.double):
    """
    Phase Lock Loop - Normalized decision-directed-costas discriminator

    Parameters
    ----------
    IP : np.double
        in-phase prompt discriminator
    QP : np.double
        quadra-phase prompt discriminator

    Returns
    -------
    phi
        phase error/misalignment [rad]
    """
    P = np.sqrt(IP**2 + QP**2)
    return QP * np.sign(IP) / P

@njit(cache=True, fastmath=True)
def PllVariance(cn0: np.double, T: np.double):
    """
    Variance in the PLL discriminator

    Parameters
    ----------
    cn0 : np.double
        Carrier to noise density ratio magnitude (not dB-Hz)
    T : np.double
        Integtation time [s]

    Returns
    -------
    np.double
        variance in the PLL discriminator [rad^2]
    """
    tmp = 1.0 / (cn0 * T)
    return tmp * (1.0 + 0.5*tmp)

# ===== FLL ====================================================================================== #

@njit(cache=True, fastmath=True)
def FllAtan2(IP_1: np.double, IP_2: np.double, QP_1: np.double, QP_2: np.double, T: np.double):
    """
    Frequency Lock Loop - ATAN2 discriminator

    Parameters
    ----------
    IP_1 : np.double
        First half in-phase prompt discriminator
    IP_2 : np.double
        Second half quadra-phase prompt discriminator
    QP_1 : np.double
        First half in-phase prompt discriminator
    QP_2 : np.double
        Second half quadra-phase prompt discriminator
    T : np.double
        Integration time for sub-correlators [s]

    Returns
    -------
    f
        frequency error/misalignment [rad/s]
    """
    x = IP_1 * QP_2 - IP_2 * QP_1
    d = IP_1 * IP_2 + QP_1 * QP_2
    return np.atan2(x, d) / T

@njit(cache=True, fastmath=True)
def FllNddcp(IP_1: np.double, IP_2: np.double, QP_1: np.double, QP_2: np.double, T: np.double):
    """
    Frequency Lock Loop - Nomralized decision-directed-cross-product discriminator

    Parameters
    ----------
    IP_1 : np.double
        First half in-phase prompt discriminator
    IP_2 : np.double
        Second half quadra-phase prompt discriminator
    QP_1 : np.double
        First half in-phase prompt discriminator
    QP_2 : np.double
        Second half quadra-phase prompt discriminator
    T : np.double
        Integration time for sub-correlators [s]

    Returns
    -------
    f
        frequency error/misalignment [rad/s]
    """
    P = np.sqrt((IP_1 + IP_2)**2 + (QP_1 + QP_2)**2)
    x = IP_1 * QP_2 - IP_2 * QP_1
    d = IP_1 * IP_2 + QP_1 * QP_2
    return x * np.sign(d) / (P * T)

@njit(cache=True, fastmath=True)
def FllVariance(cn0: np.double, T: np.double):
    """
    Variance in the FLL discriminator

    Parameters
    ----------
    cn0 : np.double
        Carrier to noise density ratio magnitude (not dB-Hz)
    T : np.double
        Integtation time [s]

    Returns
    -------
    np.double
        variance in the FLL discriminator [(rad/s)^2]
    """
    tmp = 1.0 / (cn0 * T)
    return 8.0 * tmp * (1.0 + tmp) / T**2