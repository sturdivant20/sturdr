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
from multiprocessing import Process, Queue
from dataclasses import dataclass, asdict

from pprint import pprint

from sturdr.dsp.discriminator import DllVariance
from sturdr.channel.channel import NavPacket
from sturdr.nav.ephemeris import Ephemerides, GetNavStates
from sturdr.nav.estimation import LeastSquares, LeastSquaresPos
from sturdr.utils.coordinates import ecef2lla, eci2ecef
from sturdr.utils.constants import GPS_L1CA_CARRIER_FREQ, GPS_L1CA_CODE_FREQ, LIGHT_SPEED

BETA = LIGHT_SPEED / GPS_L1CA_CODE_FREQ
LAMBDA = LIGHT_SPEED / GPS_L1CA_CARRIER_FREQ

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
    
    __slots__ = 'config', 'logger', 'log_queue', 'nav_queue', 'nav_list'
    config      : dict
    logger      : logging.Logger
    log_queue   : Queue
    nav_queue   : Queue
    
    ephemerides : list
    channel_id  : np.ndarray
    Week        : np.ndarray
    ToW         : np.ndarray
    CNo         : np.ndarray
    CodePhase   : np.ndarray
    Doppler     : np.ndarray
    
    
    def __init__(self, config: dict, log_queue: Queue, nav_queue: Queue):
        Process.__init__(self, name='SturDR_Navigator', daemon=True)
        self.config = config
        
        # find and initialize logger
        self.logger    = logging.getLogger('SturDR_Logger')
        self.log_queue = log_queue
        
        # setup nav queue
        self.nav_queue   = nav_queue
        self.ephemerides = [{}] * config['CHANNELS']['max_channels'][0]
        self.channel_id  = np.empty(config['CHANNELS']['max_channels'][0], dtype=object)
        self.Week        = np.empty(config['CHANNELS']['max_channels'][0], dtype=np.double)
        self.ToW         = np.empty(config['CHANNELS']['max_channels'][0], dtype=np.double)
        self.CNo         = np.empty(config['CHANNELS']['max_channels'][0], dtype=np.double)
        self.CodePhase   = np.empty(config['CHANNELS']['max_channels'][0], dtype=np.double)
        self.Doppler     = np.empty(config['CHANNELS']['max_channels'][0], dtype=np.double)
        return
    
    def run(self):
        while True:
            msg = self.nav_queue.get()
            if isinstance(msg, NavPacket):
                # this packet should update cour current navigation observables
                self.channel_id[msg.ChannelNum] = msg.ID
                self.Week[msg.ChannelNum]       = msg.Week
                self.ToW[msg.ChannelNum]        = msg.ToW
                self.CNo[msg.ChannelNum]        = msg.CNo
                self.CodePhase[msg.ChannelNum]  = msg.CodePhase
                self.Doppler[msg.ChannelNum]    = msg.Doppler
            elif isinstance(msg, Ephemerides):
                # this packet should update our current navigation ephemeris
                channel_num = np.where(self.channel_id == msg.id)[0][0]
                # print(self.channel_id, msg.id, channel_num)
                # pprint(asdict(msg, dict_factory=Ephemerides.dict_factory))
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
        transmit_time = self.ToW + self.CodePhase / GPS_L1CA_CODE_FREQ
        
        # calculate satellite clock, positions, and velocities
        sv_clk = np.nan * np.ones((N, 3), dtype=np.double)
        sv_pos = np.nan * np.ones((N, 3), dtype=np.double)
        sv_vel = np.nan * np.ones((N, 3), dtype=np.double)
        tgd    = np.nan * np.ones(N, dtype=np.double)
        for i in range(N):
            if mask[i]:
                sv_clk[i,:], sv_pos[i,:], sv_vel[i,:], _ = GetNavStates(transmit_time=transmit_time[i], **self.ephemerides[i])
                tgd[i] = self.ephemerides[i]['tgd']
                
        # calculate pseudorange
        receive_time = transmit_time.max() + self.config['MEASUREMENTS']['nominal_transit_time']
        psr = (receive_time - transmit_time + sv_clk[:,0] + tgd)
        
        # correct satellite positions
        for i in range(N):
            if mask[i]:
                sv_pos[i,:] = eci2ecef(psr[i], sv_pos[i,:])
                
        # create weighting matrix
        R = np.diag(BETA**2 * DllVariance(self.CNo[mask], 0.02))
        
        # least squares solution
        x, _, _ = LeastSquaresPos(sv_pos, psr * LIGHT_SPEED, R, np.zeros(4))
        
        # log results
        lla = ecef2lla(x[:3])
        self.logger.info(f"LLA => {lla[0]:.8f}{chr(176)}, {lla[1]:.8f}{chr(176)}, {lla[2]:.3f} m")
        tmp = NavResult(
            GpsWeek = self.Week[mask].max(),
            GpsToW  = self.ToW[mask].max(),
            x       = x[0],
            y       = x[1],
            z       = x[2],
            vx      = 0.0,
            vy      = 0.0,
            vz      = 0.0,
            bias    = x[3],
            drift   = 0.0,
        )
        self.log_queue.put(tmp)
        return