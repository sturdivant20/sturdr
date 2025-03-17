"""

Discriminator
=============

Standard satellite tracking match filter discriminators.
"""

from __future__ import annotations
import typing

__all__ = [
    "DllNneml",
    "DllNneml2",
    "DllVariance",
    "FllAtan2",
    "FllVariance",
    "PllAtan2",
    "PllCostas",
    "PllVariance",
]

@typing.overload
def DllNneml(E: complex, L: complex) -> float:
    """
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
    """

@typing.overload
def DllNneml(IE: float, QE: float, IL: float, QL: float) -> float:
    """
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
    """

@typing.overload
def DllNneml2(E: complex, L: complex) -> float:
    """
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
    """

@typing.overload
def DllNneml2(IE: float, QE: float, IL: float, QL: float) -> float:
    """
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
    """

def DllVariance(cno: float, T: float) -> float:
    """
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
    """

@typing.overload
def FllAtan2(P1: complex, P2: complex, T: float) -> float:
    """
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
    """

@typing.overload
def FllAtan2(IP1: float, QP1: float, IP2: float, QP2: float, T: float) -> float:
    """
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
    """

def FllVariance(cno: float, T: float) -> float:
    """
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
    """

@typing.overload
def PllAtan2(P: complex) -> float:
    """
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
    """

@typing.overload
def PllAtan2(IP: float, QP: float) -> float:
    """
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
    """

@typing.overload
def PllCostas(P: complex) -> float:
    """
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
    """

@typing.overload
def PllCostas(IP: float, QP: float) -> float:
    """
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
    """

def PllVariance(cno: float, T: float) -> float:
    """
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
    """
