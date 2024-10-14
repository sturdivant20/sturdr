"""**acquisition.py**

======  ============================================================================================
file    sturdr/dsp/acquisition.py
brief   Satellite acquisition methods. 
date    October 2024
refs    1. "Understanding GPS/GNSS Principles and Applications", 3rd Edition, 2017 
            - Kaplan & Hegarty
        2. "A Software-Defined GPS and Galileo Receiver: A Single-Frequency Approach", 2007
            - Borre, Akos, Bertelsen, Rinder, Jensen
======  ============================================================================================
"""

import numpy as np
from numba import jit, njit
from sturdr.dsp.gnss_signal import CodeNCO, CarrierNCO, shift
from sturdr.utils.constants import TWO_PI

@njit(cache=True, fastmath=True)
def SerialSearch(rfdata: np.ndarray, 
                 code: np.ndarray, 
                 doppler_range: np.double, 
                 doppler_step: np.double, 
                 sampling_freq: np.double, 
                 code_freq: np.double,
                 intermediate_freq: np.double):
    """
    Implementation of the serial acquisition method (very slow), limited to 1 ms

    Parameters
    ----------
    rfdata : np.ndarray
        Data samples recorded by the RF front end
    code : np.ndarray
        Local code (not upsampled)
    doppler_range : np.double
        Max doppler frequency to search [Hz]
    doppler_step : np.double
        Frequency step for doppler search [Z]
    sampling_freq : np.double
        Front end sampleing frequency [Hz]
    code_freq : np.double
        GNSS signal code frequency [Hz]
    intermediate_freq : np.double
        Intermediate frequency of the RF signal [Hz]

    Returns
    -------
    correlation_map : np.ndarray
        2D correlation results
    """
    
    # Initialize
    doppler_bins = np.arange(-doppler_range, doppler_range+1, doppler_step)
    upsampled_code, _ = CodeNCO(code, sampling_freq, code_freq, code.size)
    samples_per_code = upsampled_code.size
    phase_points = TWO_PI * np.arange(samples_per_code) / sampling_freq
    
    # shifting by 1/2 chips
    code_phase_shifts = 2*code.size
    correlation_map = np.zeros((doppler_bins.size, code_phase_shifts), dtype=np.double) 
    samples_per_shift = np.round(samples_per_code / code_phase_shifts).astype(int)
    
    # Doppler shift loop
    i = 0
    for doppler in doppler_bins:
        # replicate carrier
        carrier = np.exp(-1j * (intermediate_freq + doppler) * phase_points)
        signal = rfdata * carrier
        
        # shift code
        j = 0
        for sample_shift in np.arange(0, samples_per_code, samples_per_shift, dtype=int):
            temp_code = shift(upsampled_code, sample_shift)
            i_signal = np.real(signal) * temp_code
            q_signal = np.imag(signal) * temp_code
            
            correlation_map[i,j] += np.sum(i_signal)**2 + np.sum(q_signal)**2
            
            j += 1
            
        i += 1
    
    return correlation_map

# @njit(cache=True, fastmath=True) # Fails because of np.fft.fft
def PcpsSearch(rfdata: np.ndarray, 
               code: np.ndarray, 
               doppler_range: np.double, 
               doppler_step: np.double, 
               sampling_freq: np.double, 
               code_freq: np.double,
               intermediate_freq: np.double,
               coherent_integration: int=1,
               non_coherent_integration: int=1):
    """
    Implementation of the parallel code phase search (PCPS) method

    Parameters
    ----------
    rfdata : np.ndarray
        Data samples recorded by the RF front end
    code : np.ndarray
        Local code (not upsampled)
    doppler_range : np.double
        Max doppler frequency to search [Hz]
    doppler_step : np.double
        Frequency step for doppler search [Z]
    sampling_freq : np.double
        Front end sampleing frequency [Hz]
    code_freq : np.double
        GNSS signal code frequency [Hz]
    intermediate_freq : np.double
        Intermediate frequency of the RF signal [Hz]
    coherent_integration : int, optional
        number of coherent integrations to perform, by default 1
    non_coherent_integration : int, optional
        number of non-coherent periods to accumulate, by default 1

    Returns
    -------
    correlation_map : np.ndarray
        2D correlation results
    """
    # Initialize
    doppler_bins = np.arange(-doppler_range, doppler_range+1, doppler_step)
    upsampled_code, _ = CodeNCO(code, sampling_freq, code_freq, code.size)
    samples_per_code = upsampled_code.size
    correlation_map = np.zeros((doppler_bins.size, samples_per_code), dtype=np.double) 
    
    # create carrier points matrix and conj(fft(code))
    phase_points = TWO_PI * np.arange(samples_per_code) / sampling_freq
    carrier = np.exp(-1j * np.outer((intermediate_freq + doppler_bins), phase_points)) # CarrierNCO
    code_fft = np.conj(np.fft.fft(upsampled_code))
    
    # loop through non-coherent periods
    signal_idx = 0
    coherent_sum = np.zeros(correlation_map.shape, dtype=np.complex128)
    for i in np.arange(0, non_coherent_integration):
        
        # loop through coherent integrations
        coherent_sum[...] = 0.0
        for j in np.arange(0, coherent_integration):
            signal_fft = np.fft.fft(rfdata[signal_idx:signal_idx+samples_per_code] * carrier, axis=1)
            coherent_sum += np.fft.ifft(signal_fft * code_fft, axis=1)
            signal_idx += samples_per_code
            
        # sum powers non-coherently
        correlation_map += np.abs(coherent_sum)**2
    
    return correlation_map

# @njit(cache=True, fastmath=True) # fails because of unravel_index
def Peak2PeakComparison(correlation_map: np.ndarray, samples_per_code: int, samples_per_chip: int):
    """
    Compares the two highest peaks of the correlation map

    Parameters
    ----------
    correlation_map : np.ndarray
        2D-array from correlation method
    samples_per_code : int
        Number of samples per code
    samples_per_chip : int
        Number of code samples per code chip

    Returns
    -------
    first_peak_idx : np.ndarray
        Indexes of hihgest correlation peak
    acquisition_metric : np.double
        Ratio between the highest and second highest peaks
    """
    
    # Find highest correlation peak
    first_peak_idx = np.array(np.unravel_index(correlation_map.argmax(), correlation_map.shape)).astype(np.int32)
    peak_1 = correlation_map[first_peak_idx[0], first_peak_idx[1]]
    
    # Find second highest correlation peak
    exclude = np.array([first_peak_idx[1] - samples_per_chip, first_peak_idx[1] + samples_per_chip])
    if exclude[0] < 1:
        code_range = np.arange(exclude[1], samples_per_code - 1)
    elif exclude[1] >= samples_per_code:
        code_range = np.arange(0, exclude[0])
    else:
        code_range = np.append(np.arange(0, exclude[0]), np.arange(exclude[1], samples_per_code - 1))
    peak_2 = correlation_map[first_peak_idx[0], code_range].max()
    
    # determine metric
    acquisition_metric = peak_1 / peak_2
    
    return first_peak_idx, acquisition_metric

# @jit(cache=True, fastmath=True) # fails because of unravel_index
def Peak2NoiseFloorComparison(correlation_map: np.ndarray):
    """
    Compares the two highest peak to the noise floor of the acquisition plane

    Parameters
    ----------
    correlation_map : np.ndarray
        2D-array from correlation method

    Returns
    -------
    first_peak_idx : np.ndarray
        Indexes of hihgest correlation peak
    acquisition_metric : np.double
        Ratio between the highest and second highest peaks
    """
    
    # Find highest correlation peak
    first_peak_idx = np.array(np.unravel_index(correlation_map.argmax(), correlation_map.shape)).astype(np.int32)
    peak_1 = correlation_map[first_peak_idx[0], first_peak_idx[1]]
    
    # get mean and standard deviation of noise floor
    mu = correlation_map.mean()
    sigma = correlation_map.std(mean=mu)
    
    # determine metrix
    acquisition_metric = (peak_1 - mu) / sigma
    
    return first_peak_idx, acquisition_metric