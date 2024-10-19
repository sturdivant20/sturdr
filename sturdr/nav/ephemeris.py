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

# ================================================================================================ #

class KeplerianEphemeris:
    # __slots__ = 'iode', 'iodc', 'toe', 'toc', 'tgd', 'af2', 'af1', \
    #             'af0', 'e', 'sqrtA', 'deltan', 'm0', 'omega0', 'omega', 'omegaDot', 'i0', 'iDot', \
    #             'cuc', 'cus', 'cic', 'cis', 'crc', 'crs', 'ura', 'health'
    # iode        : np.double     # Issue of data Ephemeris
    # iodc        : np.double     # Issue of data Clock
    # toe         : np.double     # Time of Ephemeris
    # toc         : np.double     # Time of clock
    # tgd         : np.double     # Time, group delay
    # af2         : np.double     # 2nd order clock correction coefficient
    # af1         : np.double     # 1st order clock correction coefficient
    # af0         : np.double     # 0th order clock correction coefficient
    # e           : np.double     # Eccentricity
    # sqrtA       : np.double     # Square root of semi-major axis
    # deltan      : np.double     # Mean motion difference
    # m0          : np.double     # Mean anomaly
    # omega0      : np.double     # Longitude of ascending node
    # omega       : np.double     # Argument of perigee
    # omegaDot    : np.double     # Rate of right ascension
    # i0          : np.double     # Inclination angle
    # iDot        : np.double     # Rate of inclination angle
    # cuc         : np.double     # Cos-harmonic correction coefficient to the argument of latitude
    # cus         : np.double     # Sin-harmonic correction coefficient to the argument of latitude
    # cic         : np.double     # Cos-harmonic correction coefficient to the angle of inclination
    # cis         : np.double     # Sin-harmonic correction coefficient to the angle of inclination
    # crc         : np.double     # Cos-harmonic correction coefficient to the orbit radius
    # crs         : np.double     # Sin-harmonic correction coefficient to the orbit radius
    # ura         : np.double     # Estimated accuracy
    # health      : np.double     # Satellite health
    
    __slots__ = 'ephemerides'
    ephemerides : Ephemerides

    def __init__(self):
        self.ephemerides = Ephemerides()
        return
    
    def SetID(self, id: str):
        self.ephemerides.id = id
        return

    def UpdateParameters(self, **kwargs):
        """
        Updates the set of ephemeris parameters based on user inputs

        Raises
        ------
        ValueError
            Invalid Keplerian Ephemeris parameter
        """
        for k,v in kwargs.items():
            match k.casefold():
                case "iode":
                    self.iode = v
                case "iodc":
                    self.iodc = v
                case "toe":
                    self.toe = v
                case "toc":
                    self.toc = v
                case "tgd":
                    self.tgd = v
                case "af2":
                    self.af2 = v
                case "af1":
                    self.af1 = v
                case "af0":
                    self.af0 = v
                case "e":
                    self.e = v
                case "sqrta":
                    self.sqrtA = v
                case "deltan":
                    self.deltan = v
                case "m0":
                    self.m0 = v
                case "omega0":
                    self.omega0 = v
                case "omega":
                    self.omega = v
                case "omegadot":
                    self.omegaDot = v
                case "i0":
                    self.i0 = v
                case "idot":
                    self.iDot = v
                case "cuc":
                    self.cuc = v
                case "cus":
                    self.cus = v
                case "cic":
                    self.cic = v
                case "cis":
                    self.cis = v
                case "crc":
                    self.crc = v
                case "crs":
                    self.crs = v
                case "ura":
                    self.ura = v
                case "health":
                    self.health = v
                case _:
                    raise ValueError(f"Invalid Keplerian Ephemeris parameter!")
        return

    def GetNavStates(self, transmit_time: np.double, calc_accel: bool=False):
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
        pos : np.ndarray
            Satellite position [m]
        vel : np.ndarray
            Satellite velocity [m]
        acc : np.ndarray
            Satellite acceleratio [m]
        """
        clk, pos, vel, acc = \
            GetNavStates(self.ephemerides.toe     , # Time of Ephemeris
                         self.ephemerides.toc     , # Time of clock
                         self.ephemerides.tgd     , # Time, group delay
                         self.ephemerides.af2     , # 2nd order clock correction coefficient
                         self.ephemerides.af1     , # 1st order clock correction coefficient
                         self.ephemerides.af0     , # 0th order clock correction coefficient
                         self.ephemerides.e       , # Eccentricity
                         self.ephemerides.sqrtA   , # Square root of semi-major axis
                         self.ephemerides.deltan  , # Mean motion difference
                         self.ephemerides.m0      , # Mean anomaly
                         self.ephemerides.omega0  , # Longitude of ascending node
                         self.ephemerides.omega   , # Argument of perigee
                         self.ephemerides.omegaDot, # Rate of right ascension
                         self.ephemerides.i0      , # Inclination angle
                         self.ephemerides.iDot    , # Rate of inclination angle
                         self.ephemerides.cuc     , # Cos-harmonic correction coefficient to the argument of latitude
                         self.ephemerides.cus     , # Sin-harmonic correction coefficient to the argument of latitude
                         self.ephemerides.cic     , # Cos-harmonic correction coefficient to the angle of inclination
                         self.ephemerides.cis     , # Sin-harmonic correction coefficient to the angle of inclination
                         self.ephemerides.crc     , # Cos-harmonic correction coefficient to the orbit radius
                         self.ephemerides.crs     , # Sin-harmonic correction coefficient to the orbit radius)
                         transmit_time,
                         calc_accel
            )
        return clk, pos, vel, acc
    
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
    pos : np.ndarray
        Satellite position [m]
    vel : np.ndarray
        Satellite velocity [m]
    acc : np.ndarray
        Satellite acceleratio [m]
    """
    # constants
    A = sqrtA**2                # semi-major axis
    n0 = np.sqrt(GM / A**3)     # computed mean motion
    n = n0 + deltan             # corrected mean motion
    E2 = e**2                   # eccentricity squared
    SQ1ME2 = np.sqrt(1.0 - E2)  # common eccentricity factor
    
    # satellite clock correction (sv time)
    dt = CheckTime(transmit_time - toc)             # time from clock epoch
    dt_sv = af0 + (af1 * dt) + (af2 * dt**2) - tgd
    tk = CheckTime(transmit_time - dt_sv - toe)
    
    # mean anomaly
    Mk = np.remainder(m0 + n * tk + TWO_PI, TWO_PI)
    
    # calculate eccentric anomaly
    Ek = Mk
    for _ in range(10):
        dE = (Mk - Ek + e * np.sin(Ek)) / (1.0 - e * np.cos(Ek))
        Ek += dE
        if np.abs(dE) < 1e-15:
            break
    Ek = np.remainder(Ek + TWO_PI, TWO_PI)
    COSE = np.cos(Ek)                       # cosine of eccentric anomaly
    SINE = np.sin(Ek)                       # sine of eccentric anomaly
    DEN = 1.0 - e * COSE                    # common denominator
    
    # relativistic clock calculations (user time)
    FESQA = RELATIVISTC_F * e * sqrtA             # relativistic time factor
    ck = dt_sv + (FESQA * SINE)
    cDotk = af1 + (2.0 * af2 * dt) + (n * FESQA * COSE / DEN)
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
