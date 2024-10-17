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
from sturdr.utils.constants import (TWO_PI, WGS84_R0, OMEGA_DOT, GM, J2, OMEGA_DOT, RELATIVISTC_F, 
                                    WEEK, HALF_WEEK)
from sturdr.utils.enums import GnssSystem

# ================================================================================================ #

class KeplerianEphemeris:
    __slots__ = 'iode', 'iodc', 'toe', 'toc', 'tgd', 'af2', 'af1', \
                'af0', 'e', 'sqrtA', 'deltan', 'm0', 'omega0', 'omega', 'omegaDot', 'i0', 'iDot', \
                'cuc', 'cus', 'cic', 'cis', 'crc', 'crs', 'ura', 'health'
    iode        : np.double     # Issue of data Ephemeris
    iodc        : np.double     # Issue of data Clock
    toe         : np.double     # Time of Ephemeris
    toc         : np.double     # Time of clock
    tgd         : np.double     # Time, group delay
    af2         : np.double     # 2nd order clock correction coefficient
    af1         : np.double     # 1st order clock correction coefficient
    af0         : np.double     # 0th order clock correction coefficient
    e           : np.double     # Eccentricity
    sqrtA       : np.double     # Square root of semi-major axis
    deltan      : np.double     # Mean motion difference
    m0          : np.double     # Mean anomaly
    omega0      : np.double     # Longitude of ascending node
    omega       : np.double     # Argument of perigee
    omegaDot    : np.double     # Rate of right ascension
    i0          : np.double     # Inclination angle
    iDot        : np.double     # Rate of inclination angle
    cuc         : np.double     # Cos-harmonic correction coefficient to the argument of latitude
    cus         : np.double     # Sin-harmonic correction coefficient to the argument of latitude
    cic         : np.double     # Cos-harmonic correction coefficient to the angle of inclination
    cis         : np.double     # Sin-harmonic correction coefficient to the angle of inclination
    crc         : np.double     # Cos-harmonic correction coefficient to the orbit radius
    crs         : np.double     # Sin-harmonic correction coefficient to the orbit radius
    ura         : np.double     # Estimated accuracy
    health      : np.double     # Satellite health

    def __init__(self):
        pass

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
        # # constants
        # A = self.sqrtA**2                                       # semi-major axis
        # n0 = np.sqrt(GM / A**3)                                 # computed mean motion
        # n = n0 + self.deltan                                    # corrected mean motion
        # E2 = self.e**2                                          # eccentricity squared
        # SQ1ME2 = np.sqrt(1.0 - E2)                              # common eccentricity factor
        
        # # satellite clock correction (sv time)
        # dt = CheckTime(transmit_time - self.toc)                            # time from clock epoch
        # dt_sv = self.af0 + (self.af1 * dt) + (self.af2 * dt**2) - self.tgd
        # tk = CheckTime(transmit_time - dt_sv - self.toe)
        
        # # mean anomaly
        # Mk = np.remainder(self.m0 + n * tk + TWO_PI, TWO_PI)
        
        # # calculate eccentric anomaly
        # Ek = Mk
        # for _ in range(10):
        #     dE = (Mk - Ek + self.e * np.sin(Ek)) / (1.0 - self.e * np.cos(Ek))
        #     Ek += dE
        #     if dE < 1e-15:
        #         break
        # Ek = np.remainder(Ek + TWO_PI, TWO_PI)
        # COSE = np.cos(Ek)                                       # cosine of eccentric anomaly
        # SINE = np.sin(Ek)                                       # sine of eccentric anomaly
        # DEN = 1.0 - self.e * COSE                               # common denominator
        
        # # relativistic clock calculations (user time)
        # FESQA = RELATIVISTC_F * self.e * self.sqrtA             # relativistic time factor
        # ck = dt_sv + (FESQA * SINE)
        # cDotk = self.af1 + (2.0 * self.af2 * dt) + (n * FESQA * COSE / DEN)
        # if calc_accel:
        #     cDotDotk = 2.0 * self.af2 - (n**2 * FESQA * SINE / DEN**2)
        # else:
        #     cDotDotk = 0.0
        # clk = np.asarray([ck, cDotk, cDotDotk], dtype=np.double)
        
        # # true anomaly
        # # vk = 2.0 * np.atan2(np.sqrt((1.0 + self.e) / (1.0 - self.e)) * np.tan(0.5 * Ek), 1.0)
        # vk = 2.0 * np.atan2(SQ1ME2 * SINE, COSE - self.e)
        
        # # argument of latitude
        # Phik = np.remainder(vk + self.omega, TWO_PI)
        # COS2PHI = np.cos(2.0 * Phik)
        # SIN2PHI = np.sin(2.0 * Phik)
        
        # # corrections
        # uk = Phik + (self.cus * SIN2PHI + self.cuc * COS2PHI)                       # argument of latitude
        # rk = A * DEN + (self.crs * SIN2PHI + self.crc * COS2PHI)                    # radius
        # ik = self.i0  + self.iDot * tk + (self.cis * SIN2PHI + self.cic * COS2PHI)  # inclination
        # wk = np.remainder(self.omega0 + tk * (self.omegaDot - OMEGA_DOT)            # longitude of ascending node
        #                     - (OMEGA_DOT * self.toe) + TWO_PI, TWO_PI)              #  - (omega == w)
        # COSU = np.cos(uk)
        # SINU = np.sin(uk)
        # COSI = np.cos(ik)
        # SINI = np.sin(ik)
        # COSW = np.cos(wk)
        # SINW = np.sin(wk)
        
        # # derivatives
        # EDotk = n / DEN                                                             # eccentric anomaly rate
        # vDotk = EDotk * SQ1ME2 / DEN                                                # true anomaly rate
        # iDotk = self.iDot + 2.0 * vDotk * (self.cis * COS2PHI - self.cic * SIN2PHI) # inclination angle rate
        # uDotk = vDotk * (1.0 + 2.0 * (self.cus * COS2PHI - self.cuc * SIN2PHI))     # argument of latitude rate
        # rDotk = (self.e * A * EDotk * SINE) + 2.0 * vDotk * (self.crs * COS2PHI     # radius rate
        #             - self.crc * SIN2PHI)
        # wDotk = self.omegaDot - OMEGA_DOT                                           # longitude of ascending node rate
        
        # # position calculations
        # xk_orb = rk * COSU                              # x-position in orbital frame
        # yk_orb = rk * SINU                              # y-position in orbital frame
        # x = xk_orb * COSW - yk_orb * COSI * SINW
        # y = xk_orb * SINW + yk_orb * COSI * COSW
        # z = yk_orb * SINI
        # pos = np.asarray([x, y, z], dtype=np.double)
        
        # # velocity calculations
        # xDotk_orb = rDotk * COSU - rk * uDotk * SINU    # x-velocity in orbital frame
        # yDotk_orb = rDotk * SINU + rk * uDotk * COSU    # y-velocity in orbital frame
        # vx = -(xk_orb * wDotk * SINW) + (xDotk_orb * COSW) - (yDotk_orb * SINW * COSI) \
        #      - (yk_orb * (wDotk * COSW * COSI - iDotk * SINW * SINI))
        # vy =  (xk_orb * wDotk * COSW) + (xDotk_orb * SINW) + (yDotk_orb * COSW * COSI) \
        #      - (yk_orb * (wDotk * SINW * COSI + iDotk * COSW * SINI))
        # vz = (yDotk_orb * SINI) + (yk_orb * iDotk * COSI)
        # vel = np.asarray([vx, vy, vz], dtype=np.double)
        
        # # acceleration calculations
        # if calc_accel:
        #     F = -1.5 * J2 * (GM / rk**2) * (WGS84_R0 / rk)**2
        #     TMP1 = -GM / rk**3
        #     TMP2 = 5.0 * (z / rk)**2
        #     TMP3 = OMEGA_DOT**2
        #     ax = TMP1 * x + F * (1.0 - TMP2) * (x / rk) + 2.0 * vy * OMEGA_DOT + x * TMP3
        #     ay = TMP1 * y + F * (1.0 - TMP2) * (y / rk) - 2.0 * vx * OMEGA_DOT + y * TMP3
        #     az = TMP1 * z + F * (3.0 - TMP2) * (z / rk)
        #     acc = np.asarray([ax, ay, az], dtype=np.double)
        # else:
        #     acc = np.zeros(3, dtype=np.double)
            
        clk, pos, vel, acc = \
            GetNavStates(self.toe     , # Time of Ephemeris
                         self.toc     , # Time of clock
                         self.tgd     , # Time, group delay
                         self.af2     , # 2nd order clock correction coefficient
                         self.af1     , # 1st order clock correction coefficient
                         self.af0     , # 0th order clock correction coefficient
                         self.e       , # Eccentricity
                         self.sqrtA   , # Square root of semi-major axis
                         self.deltan  , # Mean motion difference
                         self.m0      , # Mean anomaly
                         self.omega0  , # Longitude of ascending node
                         self.omega   , # Argument of perigee
                         self.omegaDot, # Rate of right ascension
                         self.i0      , # Inclination angle
                         self.iDot    , # Rate of inclination angle
                         self.cuc     , # Cos-harmonic correction coefficient to the argument of latitude
                         self.cus     , # Sin-harmonic correction coefficient to the argument of latitude
                         self.cic     , # Cos-harmonic correction coefficient to the angle of inclination
                         self.cis     , # Sin-harmonic correction coefficient to the angle of inclination
                         self.crc     , # Cos-harmonic correction coefficient to the orbit radius
                         self.crs     , # Sin-harmonic correction coefficient to the orbit radius)
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
