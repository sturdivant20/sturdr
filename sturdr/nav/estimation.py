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

from sturdr.dsp.discriminator import DllVariance, FllVariance, PllVariance
from sturdr.nav.clock import GetNavClock
from sturdr.utils.coordinates import eci2ecef
from sturdr.utils.constants import GPS_L1CA_CARRIER_FREQ, GPS_L1CA_CODE_FREQ, LIGHT_SPEED, OMEGA_DOT

BETA = LIGHT_SPEED / GPS_L1CA_CODE_FREQ
LAMBDA = LIGHT_SPEED / GPS_L1CA_CARRIER_FREQ
OMEGA_IE_E = np.array(
    [
        [0.0, 0.0, 0.0],
        [0.0, 0.0, +OMEGA_DOT],
        [0.0, -OMEGA_DOT, 0.0]
    ], 
    dtype=np.double
)

@njit(cache=True, fastmath=True)
def LeastSquares(sv_pos: np.ndarray[np.double],
                 sv_vel: np.ndarray[np.double],
                 psr   : np.ndarray[np.double],
                 psrdot: np.ndarray[np.double],
                 CNo   : np.ndarray[np.double], 
                 x     : np.ndarray[np.double]):
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
    H[:N1, 6]       = one
    H[N1:, 7]       = one
    dz              = np.zeros(N, dtype=np.double)          # Measurement innovation
    R               = np.diag(np.concatenate((
                            BETA**2 * DllVariance(CNo, 0.02),
                            (LAMBDA/2/np.pi)**2 * FllVariance(CNo, 0.02)
                        )))

    for _ in range(20):
        for i in range(N1):
            # predict approximate range and account for earth's rotation (Groves 8.34)
            dr           = x[0:3] - sv_pos[i,:]
            r            = np.sqrt(dr @ dr)
            omegatau     = OMEGA_DOT * r / LIGHT_SPEED
            somegatau    = np.sin(omegatau)
            comegatau    = np.cos(omegatau)
            C_i_e        = np.array(
                                [
                                    [ comegatau, somegatau, 0.0],
                                    [-somegatau, comegatau, 0.0],
                                    [       0.0,       0.0, 1.0]
                                ], 
                                dtype=np.double
                           )
            
            # predict pseudorange (Groves 8.35, 9.165)
            dr           = x[0:3] - C_i_e @ sv_pos[i,:]
            r            = np.sqrt(dr @ dr) # np.sqrt(dr[0]**2 + dr[1]**2 + dr[2]**2)
            u            = dr / r
            pred_psr     = r + x[6]
            
            # predict pseudorange-rate (Groves 8.44, 9.165)
            dv           = (x[3:6] + OMEGA_IE_E @ x[0:3]) - C_i_e @ (sv_vel[i,:] + OMEGA_IE_E @ sv_pos[i,:])
            udot         = (dv * r**2 - dr * (dv @ dr)) / r**3
            pred_psrdot  = u @ dv + x[7]
            
            # setup the measurement matrix and innovation
            H[i, 0:3]    = u
            H[N1+i, 0:3] = udot
            H[N1+i, 3:6] = u
            dz[i]        = psr[i] - pred_psr
            dz[N1+i]     = psrdot[i] - pred_psrdot

        # weighted least squares solution
        P = np.linalg.inv(H.T @ R @ H)
        dx = P @ H.T @ R @ dz
        x += dx
        if np.sqrt(dx[0]**2 + dx[1]**2 + dx[2]**2) < 1e-6:
            break
    
    # print(np.array2string(H, max_line_width=80, precision=6))
    # print(np.array2string(dz, max_line_width=80, precision=6))
    return x, P


@njit(cache=True, fastmath=True)
def LeastSquaresPos(sv_pos: np.ndarray[np.double],
                    psr   : np.ndarray[np.double],
                    R     : np.ndarray[np.double], 
                    x     : np.ndarray[np.double]):
    N1              = psr.size                              # half number of measurements
    one             = np.ones(N1, dtype=np.double)          # column of ones
    H               = np.zeros((N1, 4), dtype=np.double)     # measurement matrix (Jacobian)
    H[:, 3]         = one
    dz              = np.zeros(N1, dtype=np.double)          # Measurement innovation

    for _ in range(20):
        for i in range(N1):
            # predict approximate range and account for earths rotation (Groves 8.34)
            dr           = x[0:3] - sv_pos[i,:]
            r            = np.sqrt(dr @ dr)
            omegatau     = OMEGA_DOT * r / LIGHT_SPEED
            somegatau    = np.sin(omegatau)
            comegatau    = np.cos(omegatau)
            C_i_e        = np.array(
                                [
                                    [ comegatau, somegatau, 0.0],
                                    [-somegatau, comegatau, 0.0],
                                    [       0.0,       0.0, 1.0]
                                ], 
                                dtype=np.double
                           )
            
            # predict pseudorange (Groves 8.35, 9.165)
            dr           = x[0:3] - C_i_e @ sv_pos[i,:]
            r            = np.sqrt(dr @ dr) # np.sqrt(dr[0]**2 + dr[1]**2 + dr[2]**2)
            u            = dr / r
            pred_psr     = r + x[3]
            
            # setup the measurement matrix and innovation
            H[i, 0:3]    = u
            dz[i]        = psr[i] - pred_psr

        # weighted least squares solution
        P = np.linalg.inv(H.T @ R @ H)
        dx = P @ H.T @ R @ dz
        x += dx
        if np.sqrt(dx[0]**2 + dx[1]**2 + dx[2]**2) < 1e-6:
            break

    return x, P, H


@njit(cache=True, fastmath=True)
def LeastSquaresVel(sv_vel: np.ndarray[np.double], 
                    psrdot: np.ndarray[np.double],
                    H     : np.ndarray[np.double],
                    R     : np.ndarray[np.double],
                    drift : np.double=0.0):
    if R is None:
        R = np.eye(psrdot.size)
        
    z = psrdot - (np.sum(H[:, :3] * -sv_vel, 1) + drift)
    P = np.linalg.inv(H.T @ R @ H)
    x = P @ H.T @ R @ z
    
    return x, P

class NavKF:
    """
    Kalman Filter for ECEF navigation states
    """

    __slots__ = 'x', 'P', 'A', 'Q', 'R', 'process_std', 'clock_model', 'innov_std', 'T'
    x           : np.ndarray[np.double]
    P           : np.ndarray[np.double]
    A           : np.ndarray[np.double]
    Q           : np.ndarray[np.double]
    process_std : np.double
    clock_model : str
    innov_std   : np.double
    T           : np.double
    
    def __init__(self, 
                 x: np.ndarray[np.double], 
                 P: np.ndarray[np.double], 
                 process_std: np.double, 
                 clock_model: str,
                 T: np.double):
        self.x = x
        self.P = P
        self.process_std = process_std
        self.clock_model = clock_model
        self.innov_std = 2.9
        self.T = T
        self.make_A(self.T)
        self.make_Q(self.T)
        return
    
    def run(self, 
            sv_pos: np.ndarray[np.double], 
            sv_vel: np.ndarray[np.double], 
            psr: np.ndarray[np.double], 
            psrdot: np.ndarray[np.double], 
            CNo: np.ndarray[np.double],
            T: np.double=None):

        self.x, self.P = self.Propagate(T)
        self.x, self.P, _,_ = self.Correct(sv_pos, sv_vel, psr, psrdot, CNo)
        return self.x, self.P
    
    def Propagate(self, T: np.double=None):
        if T is not None:
            self.make_A(T)
            self.make_Q(T)
        x = self.A @ self.x
        Q_2 = 0.5 * self.Q
        P = self.A @ (self.P + Q_2) @ self.A.T + Q_2
        return x, P
    
    def Correct(self, 
                sv_pos: np.ndarray[np.double], 
                sv_vel: np.ndarray[np.double], 
                psr: np.ndarray[np.double], 
                psrdot: np.ndarray[np.double], 
                CNo: np.ndarray[np.double]):
        # Correction
        N1              = psr.size                              # half number of measurements
        N               = psr.size + psrdot.size                # number of measurements
        one             = np.ones(N1, dtype=np.double)          # column of ones
        H               = np.zeros((N, 8), dtype=np.double)     # measurement matrix (Jacobian)
        H[:N1, 6]       = one
        H[N1:, 7]       = one
        dz              = np.zeros(N, dtype=np.double)          # Measurement innovation
        if N1 > 1:
            R               = np.diag(np.concatenate((
                                    BETA**2 * DllVariance(CNo, 0.02),
                                    (LAMBDA/2/np.pi)**2 * FllVariance(CNo, 0.02)
                            )))
            pred_psr        = np.zeros(N1)
            pred_psrdot     = np.zeros(N1)
            
            for i in range(N1):
                pred_psr[i], pred_psrdot[i], u, udot = PredictRangeAndRate(self.x, sv_pos[i,:], sv_vel[i,:])
                
                # setup the measurement matrix and innovation
                H[i, 0:3]    = u
                H[N1+i, 0:3] = udot
                H[N1+i, 3:6] = u
                dz[i]        = psr[i] - pred_psr[i]
                dz[N1+i]     = psrdot[i] - pred_psrdot[i]
        else:
            R = np.diag([BETA**2 * DllVariance(CNo, 0.02), (LAMBDA/2/np.pi)**2 * FllVariance(CNo, 0.02)])
            pred_psr, pred_psrdot, u, udot = PredictRangeAndRate(self.x, sv_pos, sv_vel)
            H[0, 0:3] = u
            H[1, 0:3] = udot
            H[1, 3:6] = u
            dz[0]     = psr - pred_psr
            dz[1]     = psrdot - pred_psrdot
            
        # innovation filter
        S = H @ self.P @ H.T + R
        norm_dz = np.abs(dz / np.sqrt(S.diagonal()))
        mask = norm_dz < self.innov_std
        dz = dz[mask]
        H = H[mask,:]
        R = np.diag(R.diagonal()[mask])
        
        # kalman update
        x = self.x
        P = self.P
        if dz.size > 0:
            PHt = P @ H.T
            K = PHt @ np.linalg.inv(H @ PHt + R)
            L = np.eye(8) - K @ H
            P = L @ P @ L.T + K @ R @ K.T
            x += K @ dz
        
        return x, P, pred_psr, pred_psrdot

    def make_A(self, T):
        self.A = np.asarray(
            [
                [1.0, 0.0, 0.0,   T, 0.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0,   T, 0.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0, 0.0,   T, 0.0, 0.0],
                [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,   T],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            ],
            dtype=np.double
        )
        return
    
    def make_Q(self, T):
        av = GetNavClock(self.clock_model)
        q_bb = LIGHT_SPEED**2 * (av.h0/2*T + 2*av.h1*T**2 + (2/3)*np.pi**2*av.h2*T**3)
        q_dd = LIGHT_SPEED**2 * (av.h0/(2*T) + 2*av.h1 + (8/3)*np.pi**2*av.h2*T)
        q_bd = LIGHT_SPEED**2 * (2*av.h1*T + np.pi**2*av.h2*T**2)
        q_pp = self.process_std**2 * T**3 / 3.0
        q_pv = self.process_std**2 * T**2 / 2.0
        q_vv = self.process_std**2 * T
        self.Q = np.asarray(
            [
                [q_pp, 0.0, 0.0, q_pv, 0.0, 0.0, 0.0, 0.0],
                [0.0, q_pp, 0.0, 0.0, q_pv, 0.0, 0.0, 0.0],
                [0.0, 0.0, q_pp, 0.0, 0.0, q_pv, 0.0, 0.0],
                [q_pv, 0.0, 0.0, q_vv, 0.0, 0.0, 0.0, 0.0],
                [0.0, q_pv, 0.0, 0.0, q_vv, 0.0, 0.0, 0.0],
                [0.0, 0.0, q_pv, 0.0, 0.0, q_vv, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, q_bb, q_bd],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, q_bd, q_dd],
            ]
        )
        return
    
    
@njit(cache=True, fastmath=True)
def PredictRangeAndRate(x, sv_pos, sv_vel):
    # predict approximate range and account for earth's rotation (Groves 8.34)
    dr           = x[0:3] - sv_pos
    r            = np.sqrt(dr @ dr)
    omegatau     = OMEGA_DOT * r / LIGHT_SPEED
    somegatau    = np.sin(omegatau)
    comegatau    = np.cos(omegatau)
    C_i_e        = np.asarray(
                        [
                            [ comegatau, somegatau, 0.0],
                            [-somegatau, comegatau, 0.0],
                            [       0.0,       0.0, 1.0]
                        ], 
                        dtype=np.double
                    )
    
    # predict pseudorange (Groves 8.35, 9.165)
    dr           = x[0:3] - C_i_e @ sv_pos
    r            = np.sqrt(dr @ dr) # np.sqrt(dr[0]**2 + dr[1]**2 + dr[2]**2)
    u            = dr / r
    pred_psr     = r + x[6]
    
    # predict pseudorange-rate (Groves 8.44, 9.165)
    dv           = (x[3:6] + OMEGA_IE_E @ x[0:3]) - C_i_e @ (sv_vel + OMEGA_IE_E @ sv_pos)
    udot         = (dv * r**2 - dr * (dv @ dr)) / r**3
    pred_psrdot  = u @ dv + x[7]
    
    return pred_psr, pred_psrdot, u, udot
    