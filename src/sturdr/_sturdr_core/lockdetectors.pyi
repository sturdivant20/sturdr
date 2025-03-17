"""

Lock Detectors
==============

Satellite code and carrier lock detectors.
"""

from __future__ import annotations
import typing

__all__ = ["CarrierLockDetector", "CodeLockDetector", "LockDetectors"]

class LockDetectors:
    def GetCarrierLock(self) -> bool: ...
    def GetCno(self) -> float: ...
    def GetCodeLock(self) -> bool: ...
    @typing.overload
    def Update(self, IP: float, QP: float, T: float) -> None: ...
    @typing.overload
    def Update(self, P: complex, T: float) -> None: ...
    def __init__(self, arg0: float) -> None: ...

@typing.overload
def CarrierLockDetector(
    carr_lock: bool, nbd: float, nbp: float, P: complex, alpha: float = 0.005
) -> None:
    """
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
    """

@typing.overload
def CarrierLockDetector(
    carr_lock: bool, nbd: float, nbp: float, IP: float, QP: float, alpha: float = 0.005
) -> None:
    """
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
    """

@typing.overload
def CodeLockDetector(
    code_lock: bool,
    cno: float,
    pc: float,
    pn: float,
    P_old: complex,
    P_new: complex,
    T: float,
    alpha: float = 0.005,
) -> None:
    """
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
    """

@typing.overload
def CodeLockDetector(
    code_lock: bool,
    cno: float,
    pc: float,
    pn: float,
    IP_old: float,
    QP_old: float,
    IP_new: float,
    QP_new: float,
    T: float,
    alpha: float = 0.005,
) -> None:
    """
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
    """
