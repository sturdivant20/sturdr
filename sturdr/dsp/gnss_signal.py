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
from numba import njit

# ================================================================================================ #

@njit(cache=True, fastmath=True)
def CorrelateEPL(signal: np.ndarray, code: np.ndarray, tap_spacing: int, noise_spacing: bool=False):
    """
    Generate the standard Early-Prompt-Late correlator values

    Parameters
    ----------
    signal : np.ndarray
        Recorded signal data and carrier replica multiplied together
    code : np.ndarray
        Local code replica from NCO
    tap_spacing : int
        Number of samples between each correlator

    Returns
    -------
    IE, QE, IP, QP, IL, QL, IP_1, QP_1, IP_2, QP_2 : np.double
        GNSS correlator values
    """
    len = code.size
    half_len = np.int32(len / 2)
    idx = np.arange(len, dtype=np.int32)
    
    # correlate
    E = np.sum(signal * code[(idx + tap_spacing) % len])
    L = np.sum(signal * code[(idx - tap_spacing) % len])
    # E = np.sum(signal * code[(idx + tap_spacing)])
    # L = np.sum(signal * code[(idx - tap_spacing)])
    p1 = np.sum(signal[:half_len] * code[:half_len])
    p2 = np.sum(signal[half_len:] * code[half_len:])
    P = p1 + p2
    
    if noise_spacing:
        N = np.sum(signal * code[(idx + half_len) % len])
        # N = np.sum(signal * code[(idx + noise_spacing)])
        return E.real, E.imag, P.real, P.imag, L.real, L.imag, p1.real, p1.imag, p2.real, p2.imag, \
               N.real, N.imag
    else:
        # return IE, QE, IP, QP, IL, QL, IP_1, QP_1, IP_2, QP_2
        return E.real, E.imag, P.real, P.imag, L.real, L.imag, p1.real, p1.imag, p2.real, p2.imag, \
               np.nan, np.nan

@njit(cache=True, fastmath=True)
def Correlate(signal: np.ndarray, code: np.ndarray):
    """
    Correlate a signal and code replica

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

@njit(cache=True, fastmath=True)
def CodeNCO(code: np.ndarray, 
            sampling_freq: np.double, 
            code_freq: np.double, 
            code_len: np.double, 
            remainder_phase: np.double = 0.0):
    """
    Return the upsampled version of the code provided

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
    upsampled_code = code[phase_points[:-1].astype(np.int32)]
    remainder_phase = np.mod(phase_points[-1], code_len)
    
    return upsampled_code, remainder_phase

# ================================================================================================ #

@njit(cache=True, fastmath=True)
def CarrierNCO(sampling_freq: np.double, 
               freq: np.double, 
               freq_jitter: np.double,
               size: np.double, 
               remainder_phase: np.double = 0.0):
    """
    Create a local carrier replica

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

@njit(cache=True, fastmath=True)
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

# ================================================================================================ #

@njit(cache=True, fastmath=True)
def AccumulateEPL(rfdata: np.ndarray, 
                  code: np.ndarray, 
                  tap_spacing: np.double,
                  sampling_freq: np.double, 
                  code_len: np.double,
                  code_freq: np.double,
                  carrier_freq: np.double,
                  carrier_jitter: np.double,
                  rem_code_phase: np.double, 
                  rem_carrier_phase: np.double,
                  n_samples: int,
                  samples_accumulated: int,
                  half_samples: int,
        ):
    E   = 0.0
    L   = 0.0
    N   = 0.0
    P_1 = 0.0
    P_2 = 0.0
    
    d_carrier = (carrier_freq + 0.5 * carrier_jitter / sampling_freq) / sampling_freq
    d_code    = code_freq / sampling_freq
    noise_tap = 150 # 150 chips from prompt estimate
    
    for i in range(n_samples):
        signal = np.exp(-1j * rem_carrier_phase) * rfdata[i]
        E += (code[np.int32((rem_code_phase + tap_spacing) % code_len)] * signal)
        L += (code[np.int32((rem_code_phase - tap_spacing) % code_len)] * signal)
        N += (code[np.int32((rem_code_phase + noise_tap) % code_len)] * signal)
        if samples_accumulated < half_samples:
            P_1 += (code[np.int32(rem_code_phase % code_len)] * signal)
        else:
            P_2 += (code[np.int32(rem_code_phase % code_len)] * signal)
            
        # propagate
        rem_code_phase += d_code
        rem_carrier_phase += d_carrier
        samples_accumulated += 1
    P = P_1 + P_2
    return E, P, L, P_1, P_2, N, samples_accumulated, rem_carrier_phase, rem_code_phase