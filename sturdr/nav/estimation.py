"""**navigation.py**

======  ============================================================================================
file    sturdr/nav/navigation.py 
brief   Satellite navigation techniques.
date    October 2024
refs    1. "Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems", 2nd Edition, 2013
            - Groves
        2. "Global Positioning System: Signals, Measurements, and Performance", 2nd Edition, 2006
            - Misra & Enge
======  ============================================================================================
"""

import numpy as np
from numba import njit

@njit(cache=True, fastmath=True)
def LeastSquares(sv_pos: np.ndarray[np.double],
                 sv_vel: np.ndarray[np.double],
                 psr: np.ndarray[np.double],
                 psrdot: np.ndarray[np.double],
                 R: np.ndarray[np.double] = None, 
                 x: np.ndarray[np.double] = np.zeros(8, dtype=np.double)):
    """
    Least Squares solver for GNSS position, velocity, and timing terms

    Parameters
    ----------
    sv_pos : np.ndarray[np.double]
        Satellite ECEF positions [m]
    sv_vel : np.ndarray[np.double]
        Satellite ECEF velocities [m/s]
    psr : np.ndarray[np.double]
        Pseudorange measurements [m]
    psrdot : np.ndarray[np.double]
        Pseudorange-rate measurements [m/s]
    R : np.ndarray[np.double], optional
        Measurement weighting matrix, by default None
    x : np.ndarray[np.double], optional
        Initial state estimate, by default np.zeros(8, dtype=np.double)

    Returns
    -------
    x : np.ndarray[np.double], optional
        Initial state estimate, by default np.zeros(8, dtype=np.double)
    """
    N1              = psr.size                              # half number of measurements
    N               = psr.size + psrdot.size                # number of measurements
    one             = np.ones(N1, dtype=np.double)          # column of ones
    H               = np.zeros((N, 8), dtype=np.double)     # measurement matrix (Jacobian)
    H[:N1, 6] = one
    H[N1:, 7] = one
    dz              = np.zeros(N, dtype=np.double)          # Measurement innovation
    if R is None:
        R = np.eye(N)                                       # weighting matrix

    for _ in range(10):
        for i in range(N1):
            dr           = x[:3]  - sv_pos[i, :]
            dv           = x[3:6] - sv_vel[i, :]
            r            = np.sqrt(dr[0]**2 + dr[1]**2 + dr[2]**2)
            u            = dr / r
            udot         = (dv * r**2 - dr * (dv @ dr)) / r**3
            H[i, :3]     = u
            H[N1+i, :3]  = udot
            H[N1+i, 3:6] = u
            dz[i]        = psr[i] - (r + x[6])
            dz[N1+i]     = psrdot[i] - (dv @ u + x[7])

        P = np.linalg.inv(H.T @ R @ H)
        dx = P @ H.T @ R @ dz
        x += dx
        if np.sqrt(dx[0]**2 + dx[1]**2 + dx[2]**2) < 1e-6:
            break

    return x, P


@njit(cache=True, fastmath=True)
def LeastSquaresPos(sv_pos: np.ndarray[np.double],
                    psr: np.ndarray[np.double],
                    R: np.ndarray[np.double] = None, 
                    x: np.ndarray[np.double] = np.zeros(4, dtype=np.double)):
    H       = np.zeros((psr.size, 4), dtype=np.double)
    H[:, 3] = np.ones(psr.size, dtype=np.double)
    dz      = np.zeros(psr.size, dtype=np.double)
    if R is None:
        R = np.eye(psr.size)

    for _ in range(10):
        for i in range(psr.size):
            dr       = x[:3] - sv_pos[i, :]
            r        = np.sqrt(dr[0]**2 + dr[1]**2 + dr[2]**2)
            H[i, :3] = dr / r
            dz[i]    = psr[i] - (r + x[3])

        P = np.linalg.inv(H.T @ R @ H)
        dx = P @ H.T @ R @ dz
        x += dx
        if np.sqrt(dx[0]**2 + dx[1]**2 + dx[2]**2) < 1e-6:
            break

    return x, P, H


@njit(cache=True, fastmath=True)
def LeastSquaresVel(sv_vel: np.ndarray[np.double], 
                    psrdot: np.ndarray[np.double],
                    H: np.ndarray[np.double],
                    R: np.ndarray[np.double]=None,
                    drift: np.double=0.0):
    if R is None:
        R = np.eye(psrdot.size)
        
    z = psrdot - (np.sum(H[:, :3] * -sv_vel, 1) + drift)
    P = np.linalg.inv(H.T @ R @ H)
    x = P @ H.T @ R @ z
    
    return x, P
