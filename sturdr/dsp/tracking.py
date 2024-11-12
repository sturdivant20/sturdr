"""**tracking.py**

======  ============================================================================================
file    sturdr/dsp/tracking.py
brief   Satellite scalar tracking methods.
date    October 2024
refs    1. "Understanding GPS/GNSS Principles and Applications", 3rd Edition, 2017 
            - Kaplan & Hegarty
        2. "Are PLLs Dead? A Tutorial on Kalman Filter-Based Techniques for Digital Carrier Syncronization" 
            - Vila-Valls, Closas, Navarro, Fernandez-Prades
        3. "A Software-Defined GPS and Galileo Receiver: A Single-Frequency Approach", 2007
            - Borre, Akos, Bertelsen, Rinder, Jensen
        4. "Position, Navigation, and Timing Technologies in the 21st Century", Volume 1, 2021
            - Morton, Diggelen, Spilker Jr., Parkinson
======  ============================================================================================
"""

import numpy as np
from numba import njit, float64
from numba.experimental import jitclass
from sturdr.dsp.discriminator import *

# ===== LOOP FILTERS ============================================================================= #
# See [Kaplan & Hegarty, 2017] pg. 464

@njit(cache=True, fastmath=True)
def NaturalFrequency(bw: np.double, order: int):
    """
    Calculate the natual radian frequency of the loop filter given the noise bandwidth

    Parameters
    ----------
    bw : np.double
        Noise bandwidth [Hz]
    order : int
        Loop filter order (2 or 3)

    Returns
    -------
    w0 : np.double
        Loop filter's natural radian frequency [rad/s]

    Raises
    ------
    ValueError
        Invlaid loop filter order
    """
    a = 1.4142 # 0.7071
    b = 1.1
    c = 2.4
    
    if order == 1:
        return bw / 4
    if order == 2:
        return bw * (4 * a**2) / (a**2 + 1)
    elif order == 3:
        return bw * (4 * (b*c - 1)) / (b*c**2 + (b**2 - c))
    else:
        raise ValueError(f"Loop Filter order of {order} is not valid.")

@njit(cache=True, fastmath=True)
def FLLassistedPLL_2rdOrder(phase_err: np.double, 
                            freq_err: np.double, 
                            vel_accumulator: np.double, 
                            T: np.double, 
                            w0p: np.double, 
                            w0f: np.double):
    """
    2nd order PLL/DLL assisted by 1st order FLL Digital Loop Filter

    Parameters
    ----------
    phase_err : np.double
        Phase error input (discriminator) [rad]
    freq_err : np.double
        Frequency error input (discriminator) [rad/s]
    vel_accumulator : np.double
        Velocity accumulator memory (previous value)
    T : np.double
        Coherent integration time [s]
    w0p : np.double
        Natural radian frequency of the PLL [rad/s]
    w0f : np.double
        Natural radian frequency of the FLL [rad/s]

    Returns
    -------
    output : np.double
        Doppler/Frequency esitmate from the Digital Loop Filter
    vel_accumulator : np.double
        Updated velocity accumulator memory
    """
    a = 1.4142 # 0.7071
    
    # velocity accumulator
    update = vel_accumulator + T * (phase_err * w0p**2 + freq_err * w0f)
    output = (vel_accumulator + update) * 0.5 + (phase_err * a * w0p)
    
    return output, update

@njit(cache=True, fastmath=True)
def FLLassistedPLL_3rdOrder(phase_err: np.double, 
                            freq_err: np.double, 
                            acc_accumulator: np.double, 
                            vel_accumulator: np.double, 
                            T: np.double, 
                            w0p: np.double, 
                            w0f: np.double):
    """
    3rd order PLL/DLL assisted by 2nd order FLL Digital Loop Filter

    Parameters
    ----------
    phase_err : np.double
        Phase error input (discriminator) [rad]
    freq_err : np.double
        Frequency error input (discriminator) [rad/s]
    acc_accumulator : np.double
        Acceleration accumulator memory (previous value)
    vel_accumulator : np.double
        Velocity accumulator memory (previous value)
    T : np.double
        Coherent integration time [s]
    w0p : np.double
        Natural radian frequency of the PLL [rad/s]
    w0f : np.double
        Natural radian frequency of the FLL [rad/s]

    Returns
    -------
    output : np.double
        Doppler/Frequency esitmate from the Digital Loop Filter
    acc_accumulator : np.double
        Updated acceleration accumulator memory
    vel_accumulator : np.double
        Updated velocity accumulator memory
    """
    
    a = 1.4142 # 0.7071
    b = 1.1
    c = 2.4
    
    jitter = acc_accumulator + T * (w0p**3*phase_err + w0f**2*freq_err)
    freq = vel_accumulator + T * (0.5*(acc_accumulator + jitter) + b*w0p**2*phase_err + a*w0f*freq_err)
    output = 0.5*(vel_accumulator + freq) + c*w0p*phase_err
    
    return output, jitter, freq

@njit(cache=True, fastmath=True)
def PLL_2ndOrder(phase_err: np.double, vel_accumulator: np.double, T: np.double, w0d: np.double):
    """
    2nd order Phase/Delay Lock Loop

    Parameters
    ----------
    phase_err : np.double
        Phase error input (discriminator) [rad or chip]
    vel_accumulator : np.double
        Velocity accumulator memory (previous value)
    T : np.double
        Coherent integration time [s]
    w0d : np.double
        Natural radian frequency of the DLL [rad/s]

    Returns
    -------
    _type_
        _description_
    """
    a = 1.4142 # 0.7071
    
    update = vel_accumulator + T * w0d**2*phase_err
    output = 0.5*(vel_accumulator + update) + a*w0d*phase_err 
    
    return output, update

@njit(cache=True, fastmath=True)
def PLL_3rdOrder(phase_err: np.double, 
                 acc_accumulator: np.double, 
                 vel_accumulator: np.double, 
                 T: np.double, 
                 w0p: np.double):
    """
    3nd order PLL/DLL Digital Loop Filter

    Parameters
    ----------
    phase_err : np.double
        Phase error input (discriminator) [rad or chip]
    acc_accumulator : np.double
        Acceleration accumulator memory (previous value)
    vel_accumulator : np.double
        Velocity accumulator memory (previous value)
    T : np.double
        Coherent integration time [s]
    w0p : np.double
        Natural radian frequency of the PLL [rad/s]

    Returns
    -------
    output : np.double
        Doppler/Frequency esitmate from the Digital Loop Filter
    acc_accumulator : np.double
        Updated acceleration accumulator memory
    vel_accumulator : np.double
        Updated velocity accumulator memory
    """
    b = 1.1
    c = 2.4
    
    jitter = acc_accumulator + T*w0p**3*phase_err
    freq = vel_accumulator + T * (0.5*(acc_accumulator + jitter) + b*w0p**2*phase_err)
    output = 0.5*(vel_accumulator + freq) + c*w0p*phase_err
    
    return output, jitter, freq

# ===== KALMAN FILTER ============================================================================ #

# spec = [
#     ('x', float64[:]),
#     ('u', float64[:]),
#     ('P', float64[:]),
#     ('A', float64[:]),
#     ('B', float64[:]),
#     ('C', float64[:]),
#     ('Q', float64[:]),
#     ('R', float64[:]),
#     ('T', float64),
#     ('cn0', float64),
#     ('kappa', float64),
#     ('w0d', float64),
#     ('w0p', float64),
#     ('w0f', float64),
# ]
# @jitclass(spec=spec)
class TrackingKF:
    __slots__ = 'x', 'u', 'P', 'A', 'B', 'C', 'Q', 'R', 'T', 'cn0', 'kappa', 'w0d', 'w0p', 'w0f'
    x     : np.ndarray[np.double]
    u     : np.ndarray[np.double]
    P     : np.ndarray[np.double]
    A     : np.ndarray[np.double]
    B     : np.ndarray[np.double]
    C     : np.ndarray[np.double]
    Q     : np.ndarray[np.double]
    R     : np.ndarray[np.double]
    T     : np.double
    cn0   : np.double
    kappa : np.double
    w0d   : np.double
    w0p   : np.double
    w0f   : np.double
    
    def __init__(self, 
                 w0p: np.double=0.0,
                 w0f: np.double=0.0,
                 w0d: np.double=0.0,
                 doppler: np.double=0.0, 
                 carrier_phase: np.double=0.0, 
                 code_phase: np.double=0.0, 
                 nominal_carrier_freq: np.double=2.0*np.pi*1575.42e6,
                 nominal_code_freq: np.double=1.023e6,
                 intermediate_freq: np.double=0.0,
                 cn0: np.double=5000.0,
                 T: np.double=0.001):
        """
        Constructor for TrackingKF class

        Parameters
        ----------
        w0p : np.double
            Natural radian frequency of the PLL [rad/s]
        w0f : np.double
            Natural radian frequency of the FLL [rad/s]
        w0d : np.double
            Natural radian frequency of the DLL [rad/s]
        doppler : np.double
            Initial doppler estimate [rad/s]
        carrier_phase : np.double, optional
            Initial carrier phase esitmate [rad], by default 0.0
        code_phase : np.double, optional
            Initial code phase estimate [chips], by default 0.0
        nominal_carrier_freq : np.double, optional
            Carrier frequency of the true signal [rad/s], by default 2*pi*1575.42e6
        nominal_code_freq : np.double, optional
            Chipping rate of the true signal [chip/s], by default 1.023e6
        intermediate_freq : np.double
            Intermediate frequency of the recorded signal [rad/s], by default 0.0
        T : np.double, optional
            Integration time [s], by default 0.001
        """
        self.w0p = w0p
        self.w0f = w0f
        self.w0d = w0d
        self.T = T
        self.cn0 = cn0
        self.UpdateCarrierAidingCoeff(nominal_code_freq, nominal_carrier_freq)
        self.__reset_A()
        self.__reset_B()
        self.__reset_Q()
        self.__reset_C()
        self.__reset_R()
        self.x = np.asarray([carrier_phase, doppler, 0.0, code_phase, 0.0])
        self.u = np.asarray([intermediate_freq, nominal_code_freq])
        self.P = np.diag([np.pi**2/3, 2*np.pi*500.0, 100.0, 0.1, 10.0])
        return
            
    def Run(self, phase_err: np.double, freq_err: np.double, chip_err: np.double):
        """
        Run the Kalman Filter DLL/PLL

        Parameters
        ----------
        phase_err : np.double
            Phase discriminator [rad]
        freq_err : np.double
            Frequency discriminator [rad/s]
        chip_err : np.double
            Chip discriminator [chips]
        
        Returns
        -------
        x : np.ndarray
            Current state estimates of PLL/DLL Kalman Filter
        """
        # Predicition
        self.x = self.A @ self.x + self.B @ self.u
        self.P = self.A @ self.P @ self.A.T + self.Q
        
        # Correction
        dy = np.asarray([phase_err, freq_err, chip_err])
        PCt = self.P @ self.C.T
        K = PCt @ np.linalg.inv(self.C @ PCt + self.R)
        self.P = (np.eye(5) - K @ self.C) @ self.P
        self.x += K @ dy
        
        return self.x
       
    def UpdateDoppler(self, doppler: np.double):
        """
        Update tracking loop carrier-to-noise density ratio

        Parameters
        ----------
        cn0 : np.double
            Carrier-to-noise density ratio magnitude (not dB-Hz)
        """
        self.x[1] = doppler
        return
         
    def UpdateCn0(self, cn0: np.double):
        """
        Update tracking loop carrier-to-noise density ratio

        Parameters
        ----------
        cn0 : np.double
            Carrier-to-noise density ratio magnitude (not dB-Hz)
        """
        self.cn0 = cn0
        self.__reset_R()
        return
    
    def UpdateIntegrationTime(self, T: np.double):
        """
        Update tracking loop integration time

        Parameters
        ----------
        T : np.double, optional
            Integration time [s]
        """
        self.T = T
        self.__reset_A()
        self.__reset_B()
        self.__reset_C()
        self.__reset_Q()
        # self.__reset_R()
        # self.P = np.diag([np.pi**2/3, 2*np.pi*500.0, 100.0, 0.1, 10.0])
        return
    
    def UpdateCarrierAidingCoeff(self, 
                                 nominal_code_freq: np.double, 
                                 nominal_carrier_freq: np.double):
        """
        Update coefficient for carrier aiding inside the DLL filter

        Parameters
        ----------
        nominal_code_freq : np.double
            Code frequency (e.g. 1.023e6 for GPS L1 C/A) [Hz]
        nominal_carrier_freq : np.double
            Carrier frequency (e.g. 1575.42e6 for GPS L1 C/A) [Hz]
        """
        self.kappa = nominal_code_freq / nominal_carrier_freq
        return
    
    def UpdateNaturalFreqs(self, w0p: np.double, w0f: np.double, w0d: np.double):
        """
        Update the natural frequencies for the equivalent loop filters

        Parameters
        ----------
        w0p : np.double
            Natural radian frequency of the PLL [rad/s]
        w0f : np.double
            Natural radian frequency of the FLL [rad/s]
        w0d : np.double
            Natural radian frequency of the DLL [rad/s]
        """
        
        self.w0p = w0p
        self.w0f = w0f
        self.w0d = w0d
        self.__reset_Q()
        return
            
    def __reset_A(self):
        T, k = self.T, self.kappa
        self.A = np.asarray(
            [
                [1.0,   T,   0.5*T**2, 0.0, 0.0],
                [0.0, 1.0,          T, 0.0, 0.0],
                [0.0, 0.0,        1.0, 0.0, 0.0],
                [0.0, k*T, 0.5*k*T**2, 1.0,   T],
                [0.0, 0.0,        0.0, 0.0, 1.0]
            ], 
            dtype=np.double
        )
        return
    
    def __reset_B(self):
        self.B = np.asarray(
            [
                [self.T,    0.0],
                [   0.0,    0.0],
                [   0.0,    0.0],
                [   0.0, self.T],
                [   0.0,    0.0]
            ],
            dtype=np.double
        )
        return
    
    def __reset_C(self):
        T, k = self.T, self.kappa
        self.C = np.asarray(
            [
                [1.0,   T/2,   T**2/6, 0.0, 0.0], 
                [0.0,   1.0,      T/2, 0.0, 0.0], 
                [0.0, k*T/2, k*T**2/6, 1.0, T/2]
            ],
            dtype=np.double
        )
        # self.C = np.asarray(
        #     [
        #         [1.0, 0.0, 0.0, 0.0, 0.0], 
        #         [0.0, 1.0, 0.0, 0.0, 0.0], 
        #         [0.0, 0.0, 0.0, 1.0, 0.0]
        #     ],
        #     dtype=np.double
        # )
        return
        
    def __reset_Q(self):
        T, w0d, w0p, w0f, k = self.T, self.w0d, self.w0p, self.w0f, self.kappa
        self.Q = np.asarray(
            [
                [     (w0f*T**5)/20 + (w0p*T**3)/3,   (T**2*(w0f*T**2 + 4*w0p))/8,   (T**3*w0f)/6,                  (T**3*k*(3*w0f*T**2 + 20*w0p))/60,            0],
                [      (w0f*T**4)/8 + (w0p*T**2)/2,          (w0f*T**3)/3 + w0p*T,   (T**2*w0f)/2,                      (T**2*k*(w0f*T**2 + 4*w0p))/8,            0],
                [                     (T**3*w0f)/6,                  (T**2*w0f)/2,          T*w0f,                                     (T**3*k*w0f)/6,            0],
                [(T**3*k*(3*w0f*T**2 + 20*w0p))/60, (T**2*k*(w0f*T**2 + 4*w0p))/8, (T**3*k*w0f)/6, (T**3*(3*w0f*T**2*k**2 + 20*w0p*k**2 + 20*w0d))/60, (T**2*w0d)/2],
                [                                0,                             0,              0,                                       (T**2*w0d)/2,        T*w0d]
            ],
            dtype=np.double
        )
        return
    
    def __reset_R(self):
        self.R = np.asarray(
            [
                [PllVariance(self.cn0, self.T), 0.0, 0.0], # [rad^2]
                [0.0, FllVariance(self.cn0, self.T), 0.0], # [(rad/s)^2]
                [0.0, 0.0, DllVariance(self.cn0, self.T)]  # [chip^2]
            ],
            dtype=np.double
        )
        return