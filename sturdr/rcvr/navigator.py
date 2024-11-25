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
from multiprocessing.synchronize import Event, Barrier
from multiprocessing.connection import Connection
from threading import Thread
from dataclasses import dataclass, asdict

from sturdr.channel.channel import NavPacket, VectorNcoUpdate
from sturdr.nav.ephemeris import Ephemerides, GetNavStates
from sturdr.nav.estimation import LeastSquares, NavKF, PredictRangeAndRate
from sturdr.utils.coordinates import ecef2lla, eci2ecef, ecef2enuDcm
from sturdr.dsp.tracking import VDLLUpdate, VFLLUpdate
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
    
    __slots__ = 'config', 'logger', 'log_queue', 'nav_queue', 'start_barrier', 'done_barrier', \
                'vt_pipes', 'vt_thread', 'vt_order', 'vt_dt', 'vt_init', 'ephemerides', 'channel_id', \
                'Week', 'ToW', 'CNo', 'CodePhase', 'Doppler', 'ChipDisc', 'FreqDisc', \
                'prev_code_phase', 'prev_ToW', 'prev_sv_clk_corr', 'prev_transmit_time', 'GpsToW', \
                'T', 'psr', 'psrdot', 'nav_initialized', 'x', 'kf', 'is_vt'
    config               : dict
    logger               : logging.Logger
    log_queue            : Queue
    nav_queue            : Queue
    start_barrier        : Barrier
    done_barrier         : Barrier
    vt_pipes             : np.ndarray[Connection]
    vt_thread            : Thread
    vt_order             : np.ndarray[int]
    vt_dt                : np.ndarray[np.double]
    vt_init              : np.ndarray[bool]
    
    ephemerides          : list
    channel_id           : np.ndarray[object]
    Week                 : np.ndarray[np.double]
    ToW                  : np.ndarray[np.double]
    CNo                  : np.ndarray[np.double]
    CodePhase            : np.ndarray[np.double]
    Doppler              : np.ndarray[np.double]
    ChipDisc             : np.ndarray[np.double]
    FreqDisc             : np.ndarray[np.double]
    
    prev_code_phase      : np.ndarray[np.double]
    prev_ToW             : np.ndarray[np.double]
    prev_sv_clk_corr     : np.ndarray[np.double] 
    prev_transmit_times  : np.ndarray[np.double] 
    
    GpsToW               : np.double
    T                    : np.double
    psr                  : np.ndarray[np.double] 
    psrdot               : np.ndarray[np.double] 
    nav_initialized      : bool
    x                    : np.ndarray[np.double]
    kf                   : NavKF
    is_vt                : bool
       
    def __init__(self, 
                 config: dict, 
                 log_queue: Queue, 
                 nav_queue: Queue,
                 vt_queue: Queue,
                 start_barrier: Barrier,
                 done_barrier: Barrier,
                 vt_pipes: np.ndarray[Connection],):
        Process.__init__(self, name='SturDR_Navigator', daemon=True)
        self.config = config
        
        # setup queues
        self.log_queue     = log_queue
        self.nav_queue     = nav_queue
        self.vt_queue      = vt_queue
        self.start_barrier = start_barrier
        self.done_barrier  = done_barrier
        self.vt_pipes      = vt_pipes
        self.is_vt         = False
        self.vt_order      = np.empty(config['CHANNELS']['max_channels'][0], dtype=int)
        self.vt_dt         = np.empty(config['CHANNELS']['max_channels'][0], dtype=np.double)
        self.vt_init       = np.zeros(config['CHANNELS']['max_channels'][0], dtype=bool)
        
        self.ephemerides  = [{}] * config['CHANNELS']['max_channels'][0]
        self.channel_id   = np.empty(config['CHANNELS']['max_channels'][0], dtype=object)
        self.Week         = np.empty(config['CHANNELS']['max_channels'][0], dtype=np.double)
        self.ToW          = np.empty(config['CHANNELS']['max_channels'][0], dtype=np.double)
        self.CNo          = np.empty(config['CHANNELS']['max_channels'][0], dtype=np.double)
        self.CodePhase    = np.empty(config['CHANNELS']['max_channels'][0], dtype=np.double)
        self.CarrierPhase = np.empty(config['CHANNELS']['max_channels'][0], dtype=np.double)
        self.Doppler      = np.empty(config['CHANNELS']['max_channels'][0], dtype=np.double)
        self.ChipDisc     = np.empty(config['CHANNELS']['max_channels'][0], dtype=np.double)
        self.FreqDisc     = np.empty(config['CHANNELS']['max_channels'][0], dtype=np.double)
        
        self.prev_code_phase    = np.zeros(config['CHANNELS']['max_channels'][0], dtype=np.double)
        self.prev_ToW           = np.zeros(config['CHANNELS']['max_channels'][0], dtype=np.double)
        self.prev_sv_clk_corr   = np.zeros(config['CHANNELS']['max_channels'][0], dtype=np.double)
        self.prev_transmit_times= np.zeros(config['CHANNELS']['max_channels'][0], dtype=np.double)
        
        self.T                  = 1.0 / self.config['MEASUREMENTS']['frequency']
        self.psr                = np.zeros(config['CHANNELS']['max_channels'][0], dtype=np.double)
        self.psrdot             = np.zeros(config['CHANNELS']['max_channels'][0], dtype=np.double)
        self.nav_initialized    = False
        self.x                  = np.zeros(8, dtype=np.double)

        # self.logger.debug(f"Navigator spawned.")
        return
    
    def __del__(self):
        # kill vt_thread
        # if self.vt_thread.is_alive():
        #     self.vt_thread.join()
        return
    
    def run(self):
        # find and initialize logger
        self.logger = logging.getLogger('SturDR_Logger')
        self.logger.setLevel(self.config['GENERAL']['log_level'])
        self.logger.addHandler(logging.handlers.QueueHandler(self.log_queue))
        
        # initialize vector tracking mode (sleeps unless utilized)
        self.vt_thread = Thread(target=self.ExecuteVT)
        self.vt_thread.start()
    
        # TODO: figure out why all tracking observables are not always updated prior to the navigation update
        while True:
            # wait for tracking channels to progress
            self.start_barrier.wait()
            self.done_barrier.wait()
            
            while not self.nav_queue.empty():
                msg = self.nav_queue.get()
                if isinstance(msg, NavPacket):
                    # this packet should update our current navigation observables
                    self.channel_id[msg.ChannelNum]   = msg.ID
                    self.Week[msg.ChannelNum]         = msg.Week
                    self.ToW[msg.ChannelNum]          = msg.ToW
                    self.CNo[msg.ChannelNum]          = msg.CNo
                    self.CodePhase[msg.ChannelNum]    = msg.CodePhase
                    self.Doppler[msg.ChannelNum]      = msg.Doppler
                elif isinstance(msg, Ephemerides):
                    # this packet should update our current navigation ephemeris
                    channel_num = np.where(self.channel_id == msg.id)[0][0]
                    self.ephemerides[channel_num] = asdict(msg, dict_factory=Ephemerides.dict_factory)
                elif msg is None:
                    # processing is finished
                    break
                else:
                    if not self.is_vt:
                        # this message is to run a regular navigation update!
                        self.ScalarNav()
                    else:
                        # this message is to order vector updates!
                        # self.vt_order = np.flip(np.argsort(self.CodePhase))
                        # self.GpsToW = self.ToW.max() + self.CodePhase.max() / GPS_L1CA_CODE_FREQ
                        self.vt_dt[self.vt_order[0]] += (0.02 - self.CodePhase[self.vt_order[0]] / GPS_L1CA_CODE_FREQ)
                        for i in range(1,self.config['CHANNELS']['max_channels'][0]):
                            self.vt_dt[self.vt_order[i]] = (self.CodePhase[self.vt_order[i-1]] - self.CodePhase[self.vt_order[i]]) / GPS_L1CA_CODE_FREQ
        return
    
    def ScalarNav(self):
        """_summary_
        """
        
        # # mask the data that has not gotten ephemeris
        # N = self.ToW.size
        # mask = np.array([bool(x) for x in self.ephemerides], dtype=bool)
        # M = mask.sum()
        # if M < 4:
        #     # not enough information to perform least squares
        #     return
        N = self.config['CHANNELS']['max_channels'][0]
        M = np.array([bool(x) for x in self.ephemerides], dtype=bool).sum()
        if M < N:
            return
        
        # calculate satellite transmit time
        transmit_times = self.ToW + self.CodePhase / GPS_L1CA_CODE_FREQ
        
        # calculate satellite clock, positions, and velocities
        sv_clk = np.nan * np.ones((N, 3), dtype=np.double)
        sv_pos = np.nan * np.ones((N, 3), dtype=np.double)
        sv_vel = np.nan * np.ones((N, 3), dtype=np.double)
        tgd    = np.nan * np.ones(N, dtype=np.double)
        for i in range(N):
            # if mask[i]:
            sv_clk[i,:], sv_pos[i,:], sv_vel[i,:], _ = GetNavStates(transmit_time=transmit_times[i], **self.ephemerides[i])
            tgd[i] = self.ephemerides[i]['tgd']
                
        if self.nav_initialized:
            # propagate pseudoranges
            # TODO: there must be a more numerically precise version of this
            self.psr += LIGHT_SPEED * (self.T                                        # predicted time difference
                                       - (transmit_times - self.prev_transmit_times) # actual transmit time difference
                                       - (sv_clk[:,0] - self.prev_sv_clk_corr)       # satellite bias difference
                                      )
            
            # propagate pseudorange-rates
            self.psrdot = -LAMBDA * self.Doppler + sv_clk[:,1] * LIGHT_SPEED
            
            # self.x, P = LeastSquares(sv_pos[mask,:], sv_vel[mask,:], self.psr[mask], self.psrdot[mask], self.CNo[mask], self.x)
            self.x, _ = self.kf.run(sv_pos, sv_vel, self.psr, self.psrdot, self.CNo)
        else:
            # calculate initial pseudorange
            receive_time = transmit_times.max() + self.config['MEASUREMENTS']['nominal_transit_time']
            self.psr = (receive_time - transmit_times + sv_clk[:,0] - tgd) * LIGHT_SPEED
            
            # calculate initial pseudorange-rate
            self.psrdot = -LAMBDA * self.Doppler + sv_clk[:,1] * LIGHT_SPEED
            
            # initial least squares solution
            self.x, P = LeastSquares(sv_pos, sv_vel, self.psr, self.psrdot, self.CNo, self.x)
            
            # remove initial clock bias
            self.psr -= self.x[6]
            self.x[6] = 0.0
            
            # print initial nav result
            lla = ecef2lla(self.x[:3])
            enuv = ecef2enuDcm(lla) @ self.x[3:6]
            self.logger.info(f"Initial LLA  => {lla[0]:.8f}{chr(176)}, {lla[1]:.8f}{chr(176)}, {lla[2]:.3f} m")
            self.logger.info(f"Initial ENUV => {enuv[0]:.6f}, {enuv[1]:.6f}, {enuv[2]:.6f} m/s")
            
            # initialize kalman filter
            self.kf = NavKF(self.x, 
                            P, 
                            self.config['MEASUREMENTS']['process_std'], 
                            self.config['MEASUREMENTS']['clock_model'], 
                            self.T
                        )
            self.nav_initialized = True
            
            #* ===== initialize vector processing if requested ===== *#
            if self.config['MEASUREMENTS']['vector_process']:
                self.is_vt = True
                for conn in self.vt_pipes:
                    conn.send(True)
                self.vt_order = np.flip(np.argsort(self.CodePhase))
                self.vt_dt[self.vt_order[0]] = self.ToW[self.vt_order[0]] + 0.02 - transmit_times[self.vt_order[0]]
                for i in range(1,self.config['CHANNELS']['max_channels'][0]):
                    self.vt_dt[self.vt_order[i]] = transmit_times[self.vt_order[i-1]] - transmit_times[self.vt_order[i]]
            else:
                for conn in self.vt_pipes:
                    conn.send(False)
                    
        self.prev_code_phase     = self.CodePhase
        self.prev_ToW            = self.ToW
        self.prev_sv_clk_corr    = sv_clk[:,0]
        self.prev_transmit_times = transmit_times
        self.GpsToW              = transmit_times.max()
        
        # log results
        # lla = ecef2lla(self.x[:3])
        # enuv = ecef2enuDcm(lla) @ self.x[3:6]
        # self.logger.debug(f"LLA  => {lla[0]:.8f}{chr(176)}, {lla[1]:.8f}{chr(176)}, {lla[2]:.3f} m")
        # self.logger.debug(f"ENUV => {enuv[0]:.6f}, {enuv[1]:.6f}, {enuv[2]:.6f} m/s")
        tmp = NavResult(
            GpsWeek = self.Week.max(),
            GpsToW  = self.GpsToW,
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
    
    def VectorNav(self, channel_num, T):
        """_summary_

        Parameters
        ----------
        channel_num : _type_
            _description_
        code_err : _type_
            _description_
        freq_err : _type_
            _description_
        """
        
        # print(f"Channel {channel_num} - Starting VectorNav (T = {1000*T:.2f} ms)")
        
        # next transmit_time
        prev_transmit_time = self.prev_transmit_times[channel_num]
        transmit_time = self.ToW[channel_num] + self.CodePhase[channel_num] / GPS_L1CA_CODE_FREQ
        prev_sv_clk = self.prev_sv_clk_corr[channel_num]
        if not self.vt_init[channel_num]:
            self.vt_init[channel_num] = True
            dT = self.ToW[channel_num] - prev_transmit_time
        else:
            dT = 0.02
        
        # Get new satellite position
        sv_clk, sv_pos, sv_vel, _ = GetNavStates(transmit_time=transmit_time, **self.ephemerides[channel_num])
        
        # propagate scalar pseudorange and pseudorange-rate
        self.psr[channel_num] += LIGHT_SPEED * (dT                                     # predicted time difference
                                                - (transmit_time - prev_transmit_time) # actual transmit time difference
                                                - (sv_clk[0] - prev_sv_clk)            # satellite bias difference
                                                )
        self.psrdot[channel_num] = -LAMBDA * self.Doppler[channel_num] + sv_clk[1] * LIGHT_SPEED
        
        #* -- combine scalar and vector discriminators --
        psr = self.psr[channel_num] + BETA * self.ChipDisc[channel_num]
        psrdot = self.psrdot[channel_num] - LAMBDA * self.FreqDisc[channel_num]
        
        # run kalman filter
        x_pred_prev, P_pred_prev = self.kf.Propagate(T)
        self.kf.x = x_pred_prev
        self.kf.P = P_pred_prev
        self.kf.x, self.kf.P, pred_psr, pred_psrdot = \
            self.kf.Correct(sv_pos, 
                            sv_vel, 
                            np.asarray(psr), 
                            np.asarray(psrdot), 
                            np.asarray(self.CNo[channel_num]))
        
        # Propagate next nav solution and satellite position
        ToW_next = self.ToW[channel_num] + 0.02
        x_pred_next,_ = self.kf.Propagate(0.02)
        sv_clk_next, sv_pos_next, sv_vel_next, _ = GetNavStates(transmit_time=ToW_next, **self.ephemerides[channel_num])
        
        #* -- calculate receive times --
        pred_psr_next, pred_psrdot_next, _,_ = PredictRangeAndRate(x_pred_next, sv_pos_next, sv_vel_next)
        t_r_pred = self.ToW[channel_num] + pred_psr / LIGHT_SPEED
        t_r_pred_next = ToW_next + pred_psr_next / LIGHT_SPEED
        
        if channel_num == 6:
            print()
            print(f"Channel {channel_num} - rem_code_phase = {self.CodePhase[channel_num]}")
            print(f"Channel {channel_num} - dT = {dT}, tt_k+1 - tt_k = {transmit_time - prev_transmit_time}")
            print(f"Channel {channel_num} - chip_err = {self.ChipDisc[channel_num]}, freq_err = {self.FreqDisc[channel_num]}")
            print(f"Channel {channel_num} - dtr = tr_k+1 - tr_k = {t_r_pred_next} - {t_r_pred} = {t_r_pred_next - t_r_pred}")
        
        # run nco updates and send to tracking loop
        update = VectorNcoUpdate(
            CodeDoppler=VDLLUpdate(t_r_pred_next, t_r_pred, self.CodePhase[channel_num], GPS_L1CA_CODE_FREQ, 0.02),
            CarrierDoppler=VFLLUpdate(pred_psrdot_next, LAMBDA)
        )
        self.vt_pipes[channel_num].send(update)
        
        # save
        self.prev_code_phase[channel_num]     = self.CodePhase[channel_num]
        self.prev_ToW[channel_num]            = self.ToW[channel_num]
        self.prev_sv_clk_corr[channel_num]    = sv_clk[0]
        self.prev_transmit_times[channel_num] = transmit_time
        self.GpsToW += T
        self.x = self.kf.x
        tmp = NavResult(
            GpsWeek = self.Week[channel_num],
            GpsToW  = self.GpsToW,
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
        # print(f"Channel {channel_num} - VectorNav done!")
        
        return
    
    def ExecuteVT(self):
        num_requests = 0
        while True:
            msg = self.vt_queue.get()
            
            if msg is None:
                break
            
            self.channel_id[msg.ChannelNum]   = msg.ID
            self.Week[msg.ChannelNum]         = msg.Week
            self.ToW[msg.ChannelNum]          = msg.ToW
            self.CNo[msg.ChannelNum]          = msg.CNo
            self.CodePhase[msg.ChannelNum]    = msg.CodePhase
            self.Doppler[msg.ChannelNum]      = msg.Doppler
            self.ChipDisc[msg.ChannelNum]     = msg.ChipDisc
            self.FreqDisc[msg.ChannelNum]     = msg.FreqDisc
            num_requests += 1
            
            if num_requests == self.config['CHANNELS']['max_channels'][0]:
                # calculate integration periods
                # print(f"Order = {self.vt_order}")
                # print(f"T = {np.array2string(self.vt_dt[self.vt_order]*1000, precision=2)}, sum = {1000 * self.vt_dt.sum()}")
                for channel_num in self.vt_order:
                    self.VectorNav(channel_num, self.vt_dt[channel_num])
                    
                # reset first integration period
                self.vt_dt[self.vt_order[0]] = 0.02 - self.vt_dt.sum()
                num_requests = 0
                # print("VT update is finished")
        return