"""**navigator.py**

======  ============================================================================================
file    sturdr/rcvr/navigator.py 
brief   Handles channel processing and dissemination of data to channels
date    October 2024
refs    1. "Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems", 2nd Edition, 2013
            - Groves
        2. "A Software-Defined GPS and Galileo Receiver: A Single-Frequency Approach", 2007
            - Borre, Akos, Bertelsen, Rinder, Jensen
        3. "Global Positioning System: Signals, Measurements, and Performance", 2nd Edition, 2006
            - Misra & Enge
======  ============================================================================================
"""

import numpy as np
import logging
import logging.handlers
from multiprocessing import Process, Queue
from dataclasses import dataclass, asdict

from sturdr.channel.channel import NavPacket
from sturdr.nav.ephemeris import Ephemerides, GetNavStates
from sturdr.nav.estimation import LeastSquares, NavKF
from sturdr.utils.coordinates import ecef2lla, eci2ecef, ecef2enuDcm
from sturdr.utils.constants import GPS_L1CA_CARRIER_FREQ, GPS_L1CA_CODE_FREQ, LIGHT_SPEED

BETA = LIGHT_SPEED / GPS_L1CA_CODE_FREQ
LAMBDA = LIGHT_SPEED / GPS_L1CA_CARRIER_FREQ
# print(f"LAMBDA = {LAMBDA}, BETA = {BETA}")

@dataclass(order=True, slots=True)
class NavResult:
    GpsWeek : np.uint16 = np.nan
    GpsToW  : np.double = np.nan
    x       : np.double = np.nan
    y       : np.double = np.nan
    z       : np.double = np.nan
    vx      : np.double = np.nan
    vy      : np.double = np.nan
    vz      : np.double = np.nan
    bias    : np.double = np.nan
    drift   : np.double = np.nan

class Navigator(Process):
    """
    Processes channel data into GNSS observables and generates position solutions
    """
    
    __slots__ = 'config', 'logger', 'log_queue', 'nav_queue', \
                'ephemerides', 'channel_id', 'Week', 'ToW', 'CNo', 'CodePhase', 'Doppler', 'psr', \
                'psrdot', 'prev_transmit_times', 'prev_sv_clk_corr', 'nav_initialized', 'x', 'kf'
    config               : dict
    logger               : logging.Logger
    log_queue            : Queue
    nav_queue            : Queue
    
    ephemerides          : list
    channel_id           : np.ndarray
    Week                 : np.ndarray
    ToW                  : np.ndarray
    CNo                  : np.ndarray
    CodePhase            : np.ndarray
    CarrierPhase         : np.ndarray
    Doppler              : np.ndarray
    
    T                    : np.double
    psr                  : np.ndarray 
    psrdot               : np.ndarray 
    prev_transmit_times  : np.ndarray 
    prev_sv_clk_corr     : np.ndarray 
    nav_initialized      : bool
    x                    : np.ndarray
    kf                   : NavKF
       
    def __init__(self, 
                 config: dict, 
                 log_queue: Queue, 
                 nav_queue: Queue):
        Process.__init__(self, name='SturDR_Navigator', daemon=True)
        self.config = config
        
        # setup queues
        self.log_queue = log_queue
        self.nav_queue = nav_queue
        
        self.ephemerides = [{}] * config['CHANNELS']['max_channels'][0]
        self.channel_id  = np.empty(config['CHANNELS']['max_channels'][0], dtype=object)
        self.Week        = np.empty(config['CHANNELS']['max_channels'][0], dtype=np.double)
        self.ToW         = np.empty(config['CHANNELS']['max_channels'][0], dtype=np.double)
        self.CNo         = np.empty(config['CHANNELS']['max_channels'][0], dtype=np.double)
        self.CodePhase   = np.empty(config['CHANNELS']['max_channels'][0], dtype=np.double)
        self.CarrierPhase= np.empty(config['CHANNELS']['max_channels'][0], dtype=np.double)
        self.Doppler     = np.empty(config['CHANNELS']['max_channels'][0], dtype=np.double)
        self.CodeDoppler = np.empty(config['CHANNELS']['max_channels'][0], dtype=np.double)
        
        self.T                  = 1.0 / self.config['MEASUREMENTS']['frequency']
        self.prev_transmit_times= np.zeros(config['CHANNELS']['max_channels'][0], dtype=np.double)
        self.prev_sv_clk_corr   = np.zeros(config['CHANNELS']['max_channels'][0], dtype=np.double)
        self.psr                = np.zeros(config['CHANNELS']['max_channels'][0], dtype=np.double)
        self.psrdot             = np.zeros(config['CHANNELS']['max_channels'][0], dtype=np.double)
        self.nav_initialized    = False
        self.x                  = np.zeros(8, dtype=np.double)

        # self.logger.debug(f"Navigator spawned.")
        return
    
    def run(self):
        # find and initialize logger
        self.logger = logging.getLogger('SturDR_Logger')
        self.logger.setLevel(logging.DEBUG)
        self.logger.addHandler(logging.handlers.QueueHandler(self.log_queue))
    
        # TODO: figure out why all tracking observables are not always updated prior to the navigation update
        while True:
            msg = self.nav_queue.get()
            if isinstance(msg, NavPacket):
                # this packet should update our current navigation observables
                self.channel_id[msg.ChannelNum]   = msg.ID
                self.Week[msg.ChannelNum]         = msg.Week
                self.ToW[msg.ChannelNum]          = msg.ToW
                self.CNo[msg.ChannelNum]          = msg.CNo
                self.CodePhase[msg.ChannelNum]    = msg.CodePhase
                self.CarrierPhase[msg.ChannelNum] = msg.CarrierPhase
                self.Doppler[msg.ChannelNum]      = msg.Doppler
            elif isinstance(msg, Ephemerides):
                # this packet should update our current navigation ephemeris
                channel_num = np.where(self.channel_id == msg.id)[0][0]
                self.ephemerides[channel_num] = asdict(msg, dict_factory=Ephemerides.dict_factory)
            elif msg is None:
                # processing is finished
                break
            else:
                # this message is to run a navigation update!
                self.navigate()
        return
    
    def navigate(self):
        # mask the data that has not gotten ephemeris
        N = self.ToW.size
        mask = np.array([bool(x) for x in self.ephemerides], dtype=bool)
        # mask = ~np.isnan(self.ToW)
        M = mask.sum()
        if M < 4:
            # not enough information to perform least squares
            return
        
        # calculate satellite transmit time
        transmit_times = self.ToW + self.CodePhase / GPS_L1CA_CODE_FREQ
        
        # calculate satellite clock, positions, and velocities
        sv_clk = np.nan * np.ones((N, 3), dtype=np.double)
        sv_pos = np.nan * np.ones((N, 3), dtype=np.double)
        sv_vel = np.nan * np.ones((N, 3), dtype=np.double)
        tgd    = np.nan * np.ones(N, dtype=np.double)
        for i in range(N):
            if mask[i]:
                sv_clk[i,:], sv_pos[i,:], sv_vel[i,:], _ = GetNavStates(transmit_time=transmit_times[i], **self.ephemerides[i])
                tgd[i] = self.ephemerides[i]['tgd']
                
        if not self.nav_initialized:
            # calculate initial pseudorange
            receive_time = transmit_times.max() + self.config['MEASUREMENTS']['nominal_transit_time']
            self.psr = (receive_time - transmit_times + sv_clk[:,0] - tgd) * LIGHT_SPEED
            
            # calculate initial pseudorange-rate
            self.psrdot = -LAMBDA * self.Doppler + sv_clk[:,1] * LIGHT_SPEED
            
            # initial least squares solution
            self.x, P = LeastSquares(sv_pos[mask,:], sv_vel[mask,:], self.psr[mask], self.psrdot[mask], self.CNo[mask], self.x)
            
            # remove initial clock bias
            self.psr -= self.x[6]
            self.x[6] = 0.0
            
            # initialize kalman filter
            self.kf = NavKF(self.x, 
                            P, 
                            self.config['MEASUREMENTS']['process_std'], 
                            self.config['MEASUREMENTS']['clock_model'], 
                            self.T
                        )
            self.nav_initialized = True
        else:
            # propagate pseudorange
            # TODO: there must be a more numerically precise version of this
            self.psr += LIGHT_SPEED * (self.T                                        # predicted time difference
                                       - (transmit_times - self.prev_transmit_times) # actual receive time difference
                                       - (sv_clk[:,0] - self.prev_sv_clk_corr)       # satellite bias difference
                                      )
            
            # propagate pseudorange-rate
            self.psrdot = -LAMBDA * self.Doppler + sv_clk[:,1] * LIGHT_SPEED
            
            # self.x, P = LeastSquares(sv_pos[mask,:], sv_vel[mask,:], self.psr[mask], self.psrdot[mask], self.CNo[mask], self.x)
            self.x, _ = self.kf.run(sv_pos[mask,:], sv_vel[mask,:], self.psr[mask], self.psrdot[mask], self.CNo[mask])
        
        # print(f"psr    = {np.array2string(self.psr, max_line_width=140, precision=3)}")
        # print(f"psrdot = {np.array2string(self.psrdot, max_line_width=140, precision=3)}")
        self.prev_transmit_times = transmit_times
        self.prev_sv_clk_corr    = sv_clk[:,0]
        
        # log results
        lla = ecef2lla(self.x[:3])
        # enuv = ecef2enuDcm(lla) @ self.x[3:6]
        self.logger.debug(f"LLA  => {lla[0]:.8f}{chr(176)}, {lla[1]:.8f}{chr(176)}, {lla[2]:.3f} m")
        # self.logger.debug(f"ENUV => {enuv[0]:.6f}, {enuv[1]:.6f}, {enuv[2]:.6f} m/s")
        tmp = NavResult(
            GpsWeek = self.Week[mask].max(),
            GpsToW  = self.ToW[mask].max(),
            x       = self.x[0],
            y       = self.x[1],
            z       = self.x[2],
            vx      = self.x[3],
            vy      = self.x[4],
            vz      = self.x[5],
            bias    = self.x[6],
            drift   = self.x[7],
        )
        self.log_queue.put(tmp)
        return