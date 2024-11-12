"""**lock_detector.py**

======  ============================================================================================
file    sturdr/dsp/lock_detector.py
brief   Standard tracking loop lock detectors.  
date    October 2024
refs    1. "Understanding GPS/GNSS Principles and Applications", 3rd Edition, 2017 
            - Kaplan & Hegarty
        2. "Global Positioning System: Theory and Applications, Volume 1", 1996
            - Spilker, Axlerad, Parkinson, Enge
        2. "Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems", 2nd Edition, 2013
            - Groves
        3. "Global Positioning System: Signals, Measurements, and Performance", 2nd Edition, 2006
            - Misra & Enge
======  ============================================================================================
"""

import numpy as np
from numba import njit

# @njit(cache=True, fastmath=True)
# def PhaseLockDetector(IP_memory: np.double, 
#                       QP_memory: np.double, 
#                       IP: np.double, 
#                       QP: np.double, 
#                       alpha: np.double=5e-3):
#     """
#     Carrier phase lock detector (Kaplan)

#     Parameters
#     ----------
#     IP_memory : np.double
#         Previous inphase prompt value
#     QP_memory : np.double
#         Previous quadraphase prompt value
#     IP : np.double
#         Inphase promt value
#     QP : np.double
#         Quadraphase prompt value
#     alpha : np.double, optional
#         Smoothing (filtering) coefficient, by default 0.001

#     Returns
#     -------
#     lock : bool
#         Carrier Phase lock result
#     I : np.double
#         New (filtered) inphase prompt value
#     Q : np.double
#         New (filtered) quadraphase prompt value
#     """
#     I = LowPassFilter(np.abs(IP), IP_memory, alpha)
#     Q = LowPassFilter(np.abs(QP), QP_memory, alpha)
    
#     # Q/I should be less than tan(15)=0.268 or tan(30)=0.577
#     lock = (Q / I) < 0.268
#     return lock, I, Q

@njit(cache=True, fastmath=True)
def PhaseLockDetector(NBD_memory: np.double, 
                      NBP_memory: np.double, 
                      IP: np.double, 
                      QP: np.double, 
                      alpha: np.double=5e-3):
    """
    Carrier phase lock detector (Spilker, Axlerad, Parkinson, Enge)

    Parameters
    ----------
    NBD_memory : np.double
        Previous narrow band difference value
    NBP_memory : np.double
        Previous narrow band power value
    IP : np.double
        Inphase promt value
    QP : np.double
        Quadraphase prompt value
    alpha : np.double, optional
        Smoothing (filtering) coefficient, by default 0.005

    Returns
    -------
    lock : bool
        Carrier Phase lock result
    I : np.double
        New (filtered) inphase prompt value
    Q : np.double
        New (filtered) quadraphase prompt value
    """
    NBD = LowPassFilter(IP**2 - QP**2, NBD_memory, alpha)
    NBP = LowPassFilter(IP**2 + QP**2, NBP_memory, alpha)
    
    # cos(2\phi) = NBD/NBP should have a phase error of \phi<15 degrees to be considered locked
    #   -> cos(2 * 15) = 0.866
    lock = (NBD / NBP) > 0.866
    return lock, NBD, NBP

@njit(cache=True, fastmath=True)
def CodeLockDetector(Amp_memory: np.double, 
                     noise_memory: np.double, 
                     IP: np.double, 
                     QP: np.double, 
                     IN: np.double, 
                     QN: np.double, 
                     dt: np.double, 
                     alpha: np.double=5e-3): 
    """
    Code lock detector and Carrier-to-Noise ratio estimator, must be reset if integration period 
    changes

    Parameters
    ----------
    Amp_memory : np.double
        Previous amplitude squared value
    noise_memory : np.double
        Previous noise power value
    IP : np.double
        Inphase promt value
    QP : np.double
        Quadraphase prompt value
    IN : np.double
        Inphase nosie correlator value
    QN : np.double
        Quadraphase noise correlator value
    dt : np.double
        _description_
    alpha : np.double, optional
        Smoothing (filtering) coefficient, by default 0.005

    Returns
    -------
    lock : bool
        Carrier Phase lock result
    cn0_mag : np.double
        Estimated carrier-to-noise ratio magnitude (not dB-Hz)
    A2 : np.double
        New (filtered) amplitude squared value
    n2 : np.double
        New (filtered) noise power value
    """
    # Easy C/N0 estimator
    A2 = LowPassFilter(IP**2 + QP**2, Amp_memory, alpha)
    n2 = LowPassFilter(IN**2 + QN**2, noise_memory, alpha)
    noise = 2.0 * n2
    cn0_mag = (A2 - noise) / (noise * dt)
    
    # cn0 should be greater than 25 dB (~317)
    lock = cn0_mag > 317.0
    return lock, cn0_mag, A2, n2

@njit(cache=True, fastmath=True)
def CodeAndCarrierLockDetectors(NBD_memory: np.double, 
                                NBP_memory: np.double, 
                                PC_memory: np.double, 
                                PN_memory: np.double,
                                IP: np.double, 
                                QP: np.double, 
                                IP_prev: np.double, 
                                QP_prev: np.double, 
                                dt: np.double, 
                                alpha: np.double=5e-3):
    """Code and carrier GNSS lock detectors

    Parameters
    ----------
    NBD_memory : np.double
        Narrow band difference memory
    NBP_memory : np.double
        Narrow band power memory
    PC_memory : np.double
        Carrier power memory
    PN_memory : np.double
        Noise power memory
    IP : np.double
        Inphase prompt correlator value
    QP : np.double
        Quadrature prompt correlator value
    IP_prev : np.double
        Previous inphase prompt correlator value
    QP_prev : np.double
        Previous quadrature prompt correlator value
    dt : np.double
        Integration time [s]
    alpha : np.double, optional
        Smoothing (filtering) coefficient, by default 5e-3
    """
    P_new = IP**2 + QP**2
    P_old = IP_prev**2 + QP_prev**2
    
    # --- carrier lock detection ---
    NBD = LowPassFilter(IP**2 - QP**2, NBD_memory, alpha)
    NBP = LowPassFilter(P_new, NBP_memory, alpha)
    
    # cos(2\phi) = NBD/NBP should have a phase error of \phi<15 degrees to be considered locked
    #   -> cos(2 * 15) = 0.866
    carrier_lock = (NBD / NBP) > 0.866
    
    # code lock detection (based on Beaulieu's Method)
    PC  = LowPassFilter(0.5 * (P_new + P_old), PC_memory, alpha)
    PN = LowPassFilter((np.sqrt(P_new) - np.sqrt(P_old))**2, PN_memory, alpha)
    
    # cn0 should be greater than 30 dB (~1000)
    cn0_mag = PC / (PN * dt) # (1 / (PN / PC)) / dt
    code_lock = cn0_mag > 1000.0
    
    return code_lock, carrier_lock, cn0_mag, NBD, NBP, PC, PN

@njit(cache=True, fastmath=True)
def LowPassFilter(new: np.double, old: np.double, alpha: np.double=5e-3):
    """
    Implementation of a moving average (low-pass) filter

    Parameters
    ----------
    new : np.double
        New value
    old : np.double
        Old value
    alpha : np.double, optional
        Forgetting coefficient, by default 0.005

    Returns
    -------
    output : np.double
        Filtered value
    """
    # output = (1.0 - alpha) * old + alpha * new
    output = old + alpha * (new - old)
    return output