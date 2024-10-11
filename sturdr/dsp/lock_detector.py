"""**lock_detector.py**

======  ============================================================================================
file    sturdr/dsp/lock_detector.py
brief   Standard tracking loop lock detectors.  
date    October 2024
refs    1. "Understanding GPS/GNSS Principles and Applications", 3rd Edition, 2017 
            - Kaplan & Hegarty
        2. "Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems", 2nd Edition, 2013
            - Groves
        3. "Global Positioning System: Signals, Measurements, and Performance", 2nd Edition, 2006
            - Misra & Enge
======  ============================================================================================
"""

import numpy as np

def PhaseLockDetector(IP_memory: np.double, 
                      QP_memory: np.double, 
                      IP: np.double, 
                      QP: np.double, 
                      alpha: np.double=0.01):
    """
    Carrier phase lock detector (Kaplan)

    Parameters
    ----------
    IP_memory : np.double
        Previous inphase prompt value
    QP_memory : np.double
        Previous quadraphase prompt value
    IP : np.double
        Inphase promt value
    QP : np.double
        Quadraphase prompt value
    alpha : np.double, optional
        Smoothing (filtering) coefficient, by default 0.001

    Returns
    -------
    lock : bool
        Carrier Phase lock result
    I : np.double
        New (filtered) inphase prompt value
    Q : np.double
        New (filtered) quadraphase prompt value
    """
    I = LowPassFilter(np.abs(IP), IP_memory, alpha)
    Q = LowPassFilter(np.abs(QP), QP_memory, alpha)
    
    # Q/I should be less than tan(15)=0.268 or tan(30)=0.577
    lock = (Q / I) < 0.268
    return lock, I, Q

def CodeLockDetector(Amp_memory: np.double, 
                     noise_memory: np.double, 
                     IP: np.double, 
                     QP: np.double, 
                     IN: np.double, 
                     QN: np.double, 
                     dt: np.double, 
                     alpha: np.double=0.01): 
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
        Smoothing (filtering) coefficient, by default 0.01

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
    # if Amp_memory > 0:
    A2 = LowPassFilter(IP**2 + QP**2, Amp_memory, alpha)
    n2 = LowPassFilter(IN**2 + QN**2, noise_memory, alpha)
    # else:
    #     A2 = IP**2 + QP**2
    #     n2 = IN**2 + QN**2
    cn0_mag = A2 / (2.0 * dt * n2)
    
    # cn0 should be greater than 25 dB (~317)
    lock = cn0_mag > 317.0
    return lock, cn0_mag, A2, n2

def LowPassFilter(new: np.double, old: np.double, alpha: np.double=0.01):
    """
    Implementation of a moving average (low-pass) filter

    Parameters
    ----------
    new : np.double
        New value
    old : np.double
        Old value
    alpha : np.double, optional
        Forgetting coefficient, by default 0.01

    Returns
    -------
    output : np.double
        Filtered value
    """
    output = (1.0 - alpha) * old + alpha * new
    return output