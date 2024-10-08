"""**gnss_signal.py**

======  ============================================================================================
file    sturdr/dsp/gnss_signal.py
brief   Common GNSS signal functions. 
date    October 2024
refs    1. "A Software-Defined GPS and Galileo Receiver: A Single-Frequency Approach", 2007
            - Borre, Akos, Bertelsen, Rinder, Jensen
        2. "Understanding GPS/GNSS Principles and Applications", 3rd Edition, 2017 
            - Kaplan & Hegarty
======  ============================================================================================
"""

import numpy as np

# ================================================================================================ #

def CorrelateEPL(rfdata: np.ndarray, carrier: np.ndarray, code: np.ndarray, tap_spacing: int):
    """Generate the standard Early-Prompt-Late correlator values

    Parameters
    ----------
    rfdata : np.ndarray
        Recorded signal data
    carrier : np.ndarray
        Local carrier replica from NCO
    code : np.ndarray
        Local code replica from NCO
    tap_spacing : int
        Number of samples between each correlator

    Returns
    -------
    IE, QE, IP, QP, IL, QL, IP_1, QP_1, IP_2, QP_2 : np.double
        GNSS correlator values
    """
    signal = rfdata * carrier
    len = code.size
    half_len = int(len / 2)
    idx = np.arange(len, dtype=int)
    
    # correlate
    E = np.sum(signal * code[(idx + tap_spacing) % len])
    L = np.sum(signal * code[(idx - tap_spacing) % len])
    p1 = np.sum(signal[:half_len] * code[:half_len])
    p2 = np.sum(signal[half_len:] * code[half_len:])
    P = p1 + p2
    
    # return IE, QE, IP, QP, IL, QL, IP_1, QP_1, IP_2, QP_2
    return E.real, E.imag, P.real, P.imag, L.real, L.imag, p1.real, p1.imag, p2.real, p2.imag

def Correlate(signal: np.ndarray, code: np.ndarray):
    """Correlate a signal and code replica

    Parameters
    ----------
    signal : np.ndarray
        Recorded signal data and carrier replica multiplied together
    code : np.ndarray
        Local code replica from NCO

    Returns
    -------
    I, Q : np.double
        GNSS correlator values
    """
    # correlate
    C = np.sum(signal * code)
    
    # return IE, QE, IP, QP, IL, QL, IP_1, QP_1, IP_2, QP_2
    return C.real, C.imag

# ================================================================================================ #

def CodeNCO(code: np.ndarray, 
            sampling_freq: np.double, 
            code_freq: np.double, 
            code_len: np.double, 
            remainder_phase: np.double = 0.0):
    """Return the upsampled version of the code provided

    Parameters
    ----------
    code : np.ndarray
        Code to upsample
    sampling_freq : np.double
        GNSS receiver front end sampling frequency [Hz]
    code_freq : np.double
        GNSS signal code frequency [Hz]
    code_len : np.double
        Length of the normal (not upsampled) code
    remainder_phase : np.double, optional
        Initial fractional phase of the code, by default 0.0

    Returns
    -------
    upsampled_code : np.ndarray
        Upsampled version of provided code
    remainder_phase : np.double
        Initial fractional phase of the code during the next period
    """
    
    samples_per_code = np.round(sampling_freq / (code_freq / code_len)) + 1
    phase_points = np.mod(remainder_phase + np.arange(samples_per_code) * code_freq / sampling_freq, code_len)
    upsampled_code = code[phase_points[:-1].astype(int)]
    remainder_phase = np.mod(phase_points[-1], code_len)
    
    return upsampled_code, remainder_phase

# ================================================================================================ #

def CarrierNCO(sampling_freq: np.double, 
               freq: np.double, 
               freq_jitter: np.double,
               size: np.double, 
               remainder_phase: np.double = 0.0):
    """Create a local carrier replica

    Parameters
    ----------
    sampling_freq : np.double
        Sampling frequency of the front end receiver [Hz]
    freq : np.double
        Current carrier frequency [rad/s]
    freq_jitter : np.double
        Current carrier frequency jitter [rad/s^2]
    size : np.double
        Desired replica length
    remainder_phase : np.double, optional
        Initial fractional phase of the carrier, by default 0.0

    Returns
    -------
    replica : np.ndarray
        Local carrier signal replica
    remainder_phase : np.double
        Initial fractional phase of the carrier during the next period
    """
    T = np.arange(size+1) / sampling_freq
    phase_points = (freq + 0.5 * freq_jitter * T) * T + remainder_phase
    replica = np.exp(-1j * phase_points[:-1])
    remainder_phase = np.remainder(phase_points[-1], 2*np.pi)
    
    return replica, remainder_phase

# ================================================================================================ #

def shift(arr, num):
    result = np.empty_like(arr)
    if num > 0:
        result[:num] = arr[-num:]
        result[num:] = arr[:-num]
    elif num < 0:
        result[num:] = arr[-num:]
        result[:num] = arr[-num:]
    else:
        result[:] = arr
    return result