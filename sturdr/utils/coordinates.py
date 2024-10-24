"""**coordinates.py**

======  ============================================================================================
file    sturdr/utils/coordinates.py 
brief   Common coordinate conversions.
date    October 2024
refs    1. "Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems", 2nd Edition, 2013
            - Groves
======  ============================================================================================
"""

import numpy as np
from numba import njit
from sturdr.utils.constants import WGS84_R0, WGS84_E2, OMEGA_DOT

@njit(cache=True, fastmath=True)
def ecef2lla(xyz: np.ndarray[np.double], degrees: bool=True):
    """
    Converts ECEF position coordinates to LLA coordinates

    Parameters
    ----------
    xyz : np.ndarray[np.double]
        Earth-centered-Earth-fixed position [m]
    degrees : bool, optional
        Determines whether result should be converted to degrees, by default True

    Returns
    -------
    lla : np.ndarray[np.double]
        Latitude, Longitude, Altitude coordiantes
    """
    x, y, z = xyz

    beta = np.sqrt(x*x + y*y)                                                   # (Groves C.18)
    a = np.sqrt(1 - WGS84_E2) * np.abs(z)
    b = WGS84_E2 * WGS84_R0
    E = (a - b) / beta                                                          # (Groves C.29)
    F = (a + b) / beta                                                          # (Groves C.30)
    P = 4 / 3 * (E * F + 1)                                                     # (Groves C.31)
    Q = 2 * (E * E - F * F)                                                     # (Groves C.32)
    D = P * P * P + Q * Q                                                       # (Groves C.33)
    V = (np.sqrt(D) - Q) ** (1 / 3) - (np.sqrt(D) + Q) ** (1 / 3)               # (Groves C.34)
    G = 0.5 * (np.sqrt(E * E + V) + E)                                          # (Groves C.35)
    T = np.sqrt(G * G + ((F - V * G) / (2 * G - E))) - G                        # (Groves C.36)
    phi = np.sign(z) * np.arctan((1 - T * T) / (2 * T * np.sqrt(1 - WGS84_E2))) # (Groves C.37)
    h = (beta - WGS84_R0 * T) * np.cos(phi) + (
        z - np.sign(z) * WGS84_R0 * np.sqrt(1 - WGS84_E2)                       # (Groves C.38)
    ) * np.sin(phi)
    lamb = np.arctan2(y, x)
    
    if degrees:
        return np.asarray([np.rad2deg(phi), np.rad2deg(lamb), h], dtype=np.double)
    else:
        return np.asarray([phi, lamb, h], dtype=np.double)

@njit(cache=True, fastmath=True)
def ecef2enuDcm(lla: np.ndarray[np.double], degrees: bool=True):
    """
    ECEF to ENU rotation matric

    Parameters
    ----------
    lla : np.ndarray[np.double]
        LLA coordinates
    degrees : bool, optional
        Determines whether result should be converted to degrees, by default True

    Returns
    -------
    C_e_n
        ECEF to ECI rotation matrix
    """
    if degrees:
        l = np.deg2rad(lla[0])
        sin_phi = np.sin(l)
        cos_phi = np.cos(l)
        l = np.deg2rad(lla[1])
        sin_lam = np.sin(l)
        cos_lam = np.cos(l)
    else:
        sin_phi = np.sin(lla[0])
        cos_phi = np.cos(lla[0])
        sin_lam = np.sin(lla[1])
        cos_lam = np.cos(lla[1])
    return np.asarray(
        [
            [          -sin_lam,            cos_lam,     0.0],
            [-sin_phi * cos_lam, -sin_phi * sin_lam, cos_phi],
            [ cos_phi * cos_lam,  cos_phi * sin_lam, sin_phi],
        ], dtype=np.double
    )

@njit(cache=True, fastmath=True)
def ecef2aer(xyzR: np.ndarray[np.double], xyzT: np.ndarray[np.double], degrees: bool=True):
    """
    Converts Earth-centered-Earth-fixed coordinates into Azimuth-Elevation-Range coordinates

    Parameters
    ----------
    xyzR : np.ndarray[np.double]
        ECEF coordinate of the receiver [m]
    xyzT : np.ndarray[np.double]
        ECEF coordinate of the target [m]
    degrees : bool, optional
        Determines whether result should be converted to degrees, by default True

    Returns
    -------
    aer : np.ndarray[np.double]
        Azimuth, Elevation, Range coordinates
    """
    lla0 = ecef2lla(xyzR, False)
    C_e_n = ecef2enuDcm(lla0, False)
    d_enu = C_e_n @ (xyzT - xyzR)
    
    r = np.sqrt(d_enu[0]**2 + d_enu[1]**2 + d_enu[2]**2)
    e = np.asin(d_enu[2] / r)
    a = np.atan2(d_enu[0], d_enu[1])
    
    if degrees:
        return np.asarray([np.rad2deg(a), np.rad2deg(e), r], dtype=np.double)
    else:
        return np.asarray([a, e, r], dtype=np.double)
    
@njit(cache=True, fastmath=True)
def eci2ecef(transit_time: np.double, sv_pos: np.ndarray[np.double]):
    """Accounts for the transit time of the signal from the satellite to the receiver

    Parameters
    ----------
    transit_time : np.double
        Transit time from satellite to receiver [s]
    sv_pos : np.ndarray[np.double]
        Ephemeris ECEF estimate of the satellite position [m]

    Returns
    -------
    sv_pos : np.ndarray[np.double]
        Corrected ECEF satellite position
    """
    OMEGAT = OMEGA_DOT * transit_time
    SOMEGAT = np.sin(OMEGAT)
    COMEGAT = np.cos(OMEGAT)
    R = np.asarray(
        [
            [ COMEGAT, SOMEGAT, 0.0],
            [-SOMEGAT, COMEGAT, 0.0],
            [     0.0,     0.0, 1.0]
        ]
    )
    sv_pos = R @ sv_pos
    return sv_pos