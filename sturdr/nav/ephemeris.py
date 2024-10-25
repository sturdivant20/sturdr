"""**ephemeris.py**

======  ============================================================================================
file    sturdr/nav/ephemeris.py
brief   Satellite ephemeris navigation module. 
date    October 2024
refs    1. "IS-GPS-200N", 2022
        2. "A Software-Defined GPS and Galileo Receiver: A Single-Frequency Approach", 2007
            - Borre, Akos, Bertelsen, Rinder, Jensen
======  ============================================================================================
"""

import numpy as np
from numba import njit
from dataclasses import dataclass, field
from sturdr.utils.constants import (TWO_PI, WGS84_R0, OMEGA_DOT, GM, J2, OMEGA_DOT, RELATIVISTC_F, 
                                    WEEK, HALF_WEEK)

# ================================================================================================ #

@dataclass(order=True, slots=True)
class Ephemerides:
    id          : str       = ''
    iode        : np.double = np.nan     # Issue of data Ephemeris
    iodc        : np.double = np.nan     # Issue of data Clock
    toe         : np.double = np.nan     # Time of Ephemeris
    toc         : np.double = np.nan     # Time of clock
    tgd         : np.double = np.nan     # Time, group delay
    af2         : np.double = np.nan     # 2nd order clock correction coefficient
    af1         : np.double = np.nan     # 1st order clock correction coefficient
    af0         : np.double = np.nan     # 0th order clock correction coefficient
    e           : np.double = np.nan     # Eccentricity
    sqrtA       : np.double = np.nan     # Square root of semi-major axis
    deltan      : np.double = np.nan     # Mean motion difference
    m0          : np.double = np.nan     # Mean anomaly
    omega0      : np.double = np.nan     # Longitude of ascending node
    omega       : np.double = np.nan     # Argument of perigee
    omegaDot    : np.double = np.nan     # Rate of right ascension
    i0          : np.double = np.nan     # Inclination angle
    iDot        : np.double = np.nan     # Rate of inclination angle
    cuc         : np.double = np.nan     # Cos-harmonic correction coefficient to the argument of latitude
    cus         : np.double = np.nan     # Sin-harmonic correction coefficient to the argument of latitude
    cic         : np.double = np.nan     # Cos-harmonic correction coefficient to the angle of inclination
    cis         : np.double = np.nan     # Sin-harmonic correction coefficient to the angle of inclination
    crc         : np.double = np.nan     # Cos-harmonic correction coefficient to the orbit radius
    crs         : np.double = np.nan     # Sin-harmonic correction coefficient to the orbit radius
    ura         : np.double = np.nan     # Estimated accuracy
    health      : np.double = np.nan     # Satellite health
    
    @staticmethod
    def dict_factory(x):
        exclude_fields = ("id", "iode", "iodc", "ura", "health")
        return {k: v for (k, v) in x if ((v is not None) and (k not in exclude_fields))}

# ================================================================================================ #
   
@njit(cache=True, fastmath=True)
def CheckTime(t):
    if t > HALF_WEEK:
        t = t - WEEK
    elif t < -HALF_WEEK:
        t = t + WEEK
    return t
        
@njit(cache=True, fastmath=True)
def GetNavStates(toe          : np.double,     # Time of Ephemeris
                 toc          : np.double,     # Time of clock
                 tgd          : np.double,     # Time, group delay
                 af2          : np.double,     # 2nd order clock correction coefficient
                 af1          : np.double,     # 1st order clock correction coefficient
                 af0          : np.double,     # 0th order clock correction coefficient
                 e            : np.double,     # Eccentricity
                 sqrtA        : np.double,     # Square root of semi-major axis
                 deltan       : np.double,     # Mean motion difference
                 m0           : np.double,     # Mean anomaly
                 omega0       : np.double,     # Longitude of ascending node
                 omega        : np.double,     # Argument of perigee
                 omegaDot     : np.double,     # Rate of right ascension
                 i0           : np.double,     # Inclination angle
                 iDot         : np.double,     # Rate of inclination angle
                 cuc          : np.double,     # Cos-harmonic correction coefficient to the argument of latitude
                 cus          : np.double,     # Sin-harmonic correction coefficient to the argument of latitude
                 cic          : np.double,     # Cos-harmonic correction coefficient to the angle of inclination
                 cis          : np.double,     # Sin-harmonic correction coefficient to the angle of inclination
                 crc          : np.double,     # Cos-harmonic correction coefficient to the orbit radius
                 crs          : np.double,     # Sin-harmonic correction coefficient to the orbit radius
                 transmit_time: np.double, 
                 calc_accel   : bool=False):
    """
    Calculates satellite position, velocity, and acceleration given Keplerian ephemeris 
    elements.

    Parameters
    ----------
    transmit_time: np.double
        GPS system/transmitter time (TOW) of the satellite (accounting for transit time from 
        satellite to receiver) [s]
    calc_accel : bool, optional
        Should acceleration be calculated?, by default False

    Returns
    -------
    clk : np.ndarray
        Satellite clock corrections [s]
    pos : np.ndarray
        Satellite position [m]
    vel : np.ndarray
        Satellite velocity [m/s]
    acc : np.ndarray
        Satellite acceleratio [m/s^2]
    """
    # constants
    A = sqrtA**2                # semi-major axis
    n0 = np.sqrt(GM / A**3)     # computed mean motion
    n = n0 + deltan             # corrected mean motion
    E2 = e**2                   # eccentricity squared
    SQ1ME2 = np.sqrt(1.0 - E2)  # common eccentricity factor
    
    # satellite clock correction (sv time)
    dt = CheckTime(transmit_time - toc)             # time from clock epoch
    dt_sv = af0 + dt * (af1 + dt * af2)             # group delay depends on frequency
    tk = CheckTime(transmit_time - dt_sv - toe)     # corrected time difference
    
    # mean anomaly
    Mk = np.remainder(m0 + n * tk + TWO_PI, TWO_PI)
    
    # calculate eccentric anomaly
    Ek = Mk
    for _ in range(10):
        COSE = np.cos(Ek)                               # cosine of eccentric anomaly
        SINE = np.sin(Ek)                               # sine of eccentric anomaly
        dE = (Mk - Ek + e * SINE) / (1.0 - e * COSE)
        if np.abs(dE) < 1e-15:
            break
        Ek += dE
    Ek = np.remainder(Ek + TWO_PI, TWO_PI)
    DEN = 1.0 - e * COSE                    # common denominator
    
    # relativistic clock calculations (user time)
    FESQA = RELATIVISTC_F * e * sqrtA             # relativistic time factor
    # ck = af0 + tk * (af1 + tk * af2) + (FESQA * SINE)
    ck = dt_sv + (FESQA * SINE)
    cDotk = af1 + (2.0 * af2 * tk) + (n * FESQA * COSE / DEN)
    if calc_accel:
        cDotDotk = 2.0 * af2 - (n**2 * FESQA * SINE / DEN**2)
    else:
        cDotDotk = 0.0
    clk = np.asarray([ck, cDotk, cDotDotk], dtype=np.double)
    
    # true anomaly
    # vk = 2.0 * np.atan2(np.sqrt((1.0 + e) / (1.0 - e)) * np.tan(0.5 * Ek), 1.0)
    vk = np.atan2(SQ1ME2 * SINE, COSE - e)
    
    # argument of latitude
    Phik = np.remainder(vk + omega, TWO_PI)
    COS2PHI = np.cos(2.0 * Phik)
    SIN2PHI = np.sin(2.0 * Phik)
    
    # corrections
    uk = Phik + (cus * SIN2PHI + cuc * COS2PHI)                 # argument of latitude
    rk = A * DEN + (crs * SIN2PHI + crc * COS2PHI)              # radius
    ik = i0  + iDot * tk + (cis * SIN2PHI + cic * COS2PHI)      # inclination
    wk = np.remainder(omega0 + tk * (omegaDot - OMEGA_DOT)      # longitude of ascending node
                        - (OMEGA_DOT * toe) + TWO_PI, TWO_PI)   #  - (omega == w)
    COSU = np.cos(uk)
    SINU = np.sin(uk)
    COSI = np.cos(ik)
    SINI = np.sin(ik)
    COSW = np.cos(wk)
    SINW = np.sin(wk)
    
    # derivatives
    EDotk = n / DEN                                                 # eccentric anomaly rate
    vDotk = EDotk * SQ1ME2 / DEN                                    # true anomaly rate
    iDotk = iDot + 2.0 * vDotk * (cis * COS2PHI - cic * SIN2PHI)    # inclination angle rate
    uDotk = vDotk * (1.0 + 2.0 * (cus * COS2PHI - cuc * SIN2PHI))   # argument of latitude rate
    rDotk = (e * A * EDotk * SINE) + 2.0 * vDotk * (crs * COS2PHI   # radius rate
                - crc * SIN2PHI)
    wDotk = omegaDot - OMEGA_DOT                                    # longitude of ascending node rate
    
    # position calculations
    xk_orb = rk * COSU                              # x-position in orbital frame
    yk_orb = rk * SINU                              # y-position in orbital frame
    x = xk_orb * COSW - yk_orb * COSI * SINW
    y = xk_orb * SINW + yk_orb * COSI * COSW
    z = yk_orb * SINI
    pos = np.asarray([x, y, z], dtype=np.double)
    
    # velocity calculations
    xDotk_orb = rDotk * COSU - rk * uDotk * SINU    # x-velocity in orbital frame
    yDotk_orb = rDotk * SINU + rk * uDotk * COSU    # y-velocity in orbital frame
    vx = -(xk_orb * wDotk * SINW) + (xDotk_orb * COSW) - (yDotk_orb * SINW * COSI) \
            - (yk_orb * (wDotk * COSW * COSI - iDotk * SINW * SINI))
    vy =  (xk_orb * wDotk * COSW) + (xDotk_orb * SINW) + (yDotk_orb * COSW * COSI) \
            - (yk_orb * (wDotk * SINW * COSI + iDotk * COSW * SINI))
    vz = (yDotk_orb * SINI) + (yk_orb * iDotk * COSI)
    vel = np.asarray([vx, vy, vz], dtype=np.double)
    
    # acceleration calculations
    if calc_accel:
        F = -1.5 * J2 * (GM / rk**2) * (WGS84_R0 / rk)**2
        TMP1 = -GM / rk**3
        TMP2 = 5.0 * (z / rk)**2
        TMP3 = OMEGA_DOT**2
        ax = TMP1 * x + F * (1.0 - TMP2) * (x / rk) + 2.0 * vy * OMEGA_DOT + x * TMP3
        ay = TMP1 * y + F * (1.0 - TMP2) * (y / rk) - 2.0 * vx * OMEGA_DOT + y * TMP3
        az = TMP1 * z + F * (3.0 - TMP2) * (z / rk)
        acc = np.asarray([ax, ay, az], dtype=np.double)
    else:
        acc = np.zeros(3, dtype=np.double)
        
    return clk, pos, vel, acc
