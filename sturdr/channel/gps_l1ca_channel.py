"""**gps_l1ca_channel.py**

======  ============================================================================================
file    sturdr/channel/gps_l1ca_channel.py
brief   Implementation of channel.py for GPS L1 C/A signals.
date    October 2024
refs    1. "Understanding GPS/GNSS Principles and Applications", 3rd Edition, 2017 
            - Kaplan & Hegarty
        2. "Global Positioning System: Signals, Measurements, and Performance", 2nd Edition, 2006
            - Misra & Enge
        3. "A Software-Defined GPS and Galileo Receiver: A Single-Frequency Approach", 2007
            - Borre, Akos, Bertelsen, Rinder, Jensen
======  ============================================================================================
"""

import numpy as np
from numba import njit
import multiprocessing
from multiprocessing import Queue, Barrier

from sturdr.channel.channel import Channel, ChannelPacket
from sturdr.dsp.acquisition import PcpsSearch, Peak2NoiseFloorComparison
from sturdr.dsp.tracking import NaturalFrequency, TrackingKF
from sturdr.dsp.gnss_signal import AccumulateEPL
from sturdr.dsp.discriminator import PllCostas, DllNneml, DllNcdp, FllAtan2
from sturdr.dsp.lock_detector import CodeLockDetector, PhaseLockDetector, CodeAndCarrierLockDetectors
from sturdr.nav.gps_lnav import GpsLnavParser
from sturdr.utils.constants import GPS_L1CA_CARRIER_FREQ, GPS_L1CA_CODE_FREQ, GPS_L1CA_CODE_SIZE
from sturdr.utils.enums import ChannelState, GnssSystem, GnssSignalTypes
from sturdr.utils.rf_data_buffer import RfDataBuffer

NP_TWO_PI = 2 * np.pi

class GpsL1caChannel(Channel):
    __slots__ = 'code', 'rem_code_phase', 'code_doppler', 'rem_carrier_phase', 'carrier_doppler', \
                'carrier_jitter', 'T', 'PDI', 'total_samples', 'half_samples', 'samples_processed', \
                'samples_remaining', 'kf', 'tap_spacing', 'IP', 'QP', 'IE', 'QE', 'IL', 'QL', \
                'IP_1', 'QP_1', 'IP_2', 'QP_2', 'IN', 'QN', 'cn0_mag', 'N0P_memory', 'NBD_memory', \
                'NBP_memory', 'min_converg_time', 'bit_sync_hist', 'lnav_parser', 'counter', \
                'samples_per_ms', 'samples_per_chip'
    
    # Replicas
    code              : np.ndarray[np.int8]     # PRN replica code
    rem_code_phase    : np.double               # remainder code phase
    code_doppler      : np.double               # current code doppler
    rem_carrier_phase : np.double               # remainder carrier phase
    carrier_doppler   : np.double               # current carrier doppler
    carrier_jitter    : np.double               # current carrier doppler rate
    T                 : np.double               # current integration time [s]
    PDI               : np.uint8                # current integration time [ms]
    total_samples     : np.uint32               # predicted number of total samples in integration period
    half_samples      : np.uint32               # half of total_samples
    samples_processed : np.uint32               # 
    samples_remaining : np.uint32               # 
    
    # Tracking
    w0p               : np.double
    w0d               : np.double
    w0f               : np.double
    kf                : TrackingKF          # code/carrier tracking filter
    tap_spacing       : np.double           # spacing between early, promt, and late correlators
    IP                : np.double           # inphase prompt
    QP                : np.double           # quadrature prompt
    IE                : np.double           # inphase early
    QE                : np.double           # quadrature early
    IL                : np.double           # inphase late
    QL                : np.double           # quadrature late
    IP_1              : np.double           # first half of inphase prompt
    QP_1              : np.double           # first half of quadrature prompt
    IP_2              : np.double           # second half of inphase prompt
    QP_2              : np.double           # second half of quadrature prompt
    IN                : np.double           # inphase noise
    QN                : np.double           # quadrature noise
    
    # Lock Detectors
    cn0_mag           : np.double           # estimated carrier-to-noise density ratio
    NBD_memory        : np.double           # smoothed narrow band difference estimate
    NBP_memory        : np.double           # smoothed narrow band power esitmate
    N0P_memory        : np.double           # smoothed noise power estimate

    # Telemetry
    min_converg_time  : np.uint16           # minimum time (ms) alloted before checking telemetry status
    bit_sync_hist     : np.ndarray[np.uint8]# histogram for determing bit sync
    lnav_parser       : GpsLnavParser       # gps ephermeris/almanac parser and sv position estimator
    counter           : np.uint64           # integration period counter
    
    # Sample constants
    samples_per_ms    : np.uint16           # samples in 1 millisecond of RF data
    samples_per_chip  : np.uint8            # samples in 1 chip of the code
    
# ------------------------------------------------------------------------------------------------ #

    def __init__(self, 
                 config: dict, 
                 cid: str, 
                 rfbuffer: RfDataBuffer, 
                 log_queue: Queue, 
                 start_barrier: multiprocessing.synchronize.Barrier, 
                 done_barrier: multiprocessing.synchronize.Barrier, 
                 num: int):
        Channel.__init__(self, config, cid, rfbuffer, log_queue, start_barrier, done_barrier, num)
        self.channel_status.Constellation = GnssSystem.GPS
        self.channel_status.Signal = GnssSignalTypes.GPS_L1CA
        self.channel_status.State = ChannelState.IDLE
        
        # sample constants
        self.samples_per_ms = int(self.config['RFSIGNAL']['sampling_freq'] / 1000)
        self.samples_per_chip = self.config['RFSIGNAL']['sampling_freq'] / GPS_L1CA_CODE_FREQ
        
        # replicas
        self.rem_carrier_phase = 0.0
        self.carrier_doppler = 0.0
        self.carrier_jitter = 0.0
        self.rem_code_phase = 0.0
        self.code_doppler = 0.0
        self.T = 0.001
        self.PDI = 1
        
        # tracking
        self.tap_spacing = self.config['TRACKING']['correlator_epl_wide'] # int(self.config['TRACKING']['correlator_epl_wide'] * self.samples_per_chip)
        self.kappa = GPS_L1CA_CODE_FREQ / (NP_TWO_PI * GPS_L1CA_CARRIER_FREQ)
        self.IP = 0.0
        self.QP = 0.0
        self.IE = 0.0
        self.QE = 0.0
        self.IL = 0.0
        self.QL = 0.0
        self.IP_1 = 0.0
        self.QP_1 = 0.0
        self.IP_2 = 0.0
        self.QP_2 = 0.0
        self.IN = 0.0
        self.QN = 0.0
        self.w0p = NaturalFrequency(self.config["TRACKING"]["pll_bandwidth_wide"], 3)
        self.w0f = NaturalFrequency(self.config["TRACKING"]["fll_bandwidth_wide"], 2)
        self.w0d = NaturalFrequency(self.config["TRACKING"]["dll_bandwidth_wide"], 2)
        self.kf = TrackingKF(self.w0p, self.w0f, self.w0d, 
                             nominal_carrier_freq=NP_TWO_PI * GPS_L1CA_CARRIER_FREQ,
                             nominal_code_freq=GPS_L1CA_CODE_FREQ,
                             intermediate_freq=NP_TWO_PI * self.config['RFSIGNAL']['intermediate_freq'],
                             T = self.T
            )
        
        # lock detectors
        self.cn0_mag      = 0.0
        self.NBD_memory   = 0.0
        self.NBP_memory   = 0.0
        self.N0P_memory   = 0.0
        
        # telemetry
        self.min_converg_time  = self.config['TRACKING']['min_converg_time_ms']
        self.bit_sync_hist     = np.zeros(20)
        self.lnav_parser       = GpsLnavParser()
        self.counter           = 0
        
        return

# ------------------------------------------------------------------------------------------------ #

    def run(self):
        # print("Run", flush=True)
        while True:
            # wait for new data to process
            self.start_barrier.wait()
            
            # update the write pointer location for this process
            #   -> necessary because `rfbuffer` is in `another Process` which cannot be seen in 
            #      `this Process`
            self.rfbuffer.UpdateWritePtr(self.samples_per_ms * self.config['GENERAL']['ms_read_size'])
            
            # process data
            if self.channel_status.State == ChannelState.TRACKING:
                self.Track()
            elif self.channel_status.State == ChannelState.ACQUIRING:
                self.Acquire()
            
            # send processed results to logging/data queue          
            self.log_queue.put(self.channel_status)
            
            # send the signal that the channel is finished      
            self.done_barrier.wait()
        return

# ------------------------------------------------------------------------------------------------ #

    def SetSatellite(self, satelliteID: np.uint8):
        """
        Set the GNSS signal and satellite tracked by the channel.
        """
        # print("SetSatellite", flush=True)
        
        self.channel_status.ID = satelliteID
        self.code = gps_l1ca_code(satelliteID)
        self.channel_status.State = ChannelState.ACQUIRING
        self.channel_status.ID = f"GPS{satelliteID}"
        
        # initialize total samples needed for acquisition
        self.total_samples  = np.uint32(self.config['ACQUISITION']['coherent_integration'] * \
                                  self.config['ACQUISITION']['non_coherent_integration'] * \
                                  self.samples_per_ms)
        self.half_samples = int(self.total_samples / 2)
        # print(f"Total Samples = {self.total_samples}")
        return

# ------------------------------------------------------------------------------------------------ #

    def Acquire(self):
        """
        Trys to acquire the set satellite
        """
        # print("Acquire", flush=True)
        # print(f"write_ptr = {self.rfbuffer.write_ptr}")
        # print(f"read_ptr = {self.buffer_ptr}")
        
        # make sure enough samples have been parsed
        if self.rfbuffer.GetNumUnreadSamples(self.buffer_ptr) < self.total_samples:
            return
        
        # run acquisition
        correlation_map = PcpsSearch(self.rfbuffer.Pull(self.buffer_ptr, self.total_samples), 
                                     self.code,
                                     self.config['ACQUISITION']['doppler_range'],
                                     self.config['ACQUISITION']['doppler_step'],
                                     self.config['RFSIGNAL']['sampling_freq'],
                                     GPS_L1CA_CODE_FREQ, 
                                     self.config['RFSIGNAL']['intermediate_freq'],
                                     self.config['ACQUISITION']['coherent_integration'],
                                     self.config['ACQUISITION']['non_coherent_integration']
                            )
        
        # test for succesfull acquisition
        first_peak_idx, acquisition_metric = Peak2NoiseFloorComparison(correlation_map)
        
        if acquisition_metric > self.config['ACQUISITION']['threshold']:
            # update states
            self.carrier_doppler = NP_TWO_PI * float(-self.config['ACQUISITION']['doppler_range'] + \
                                    first_peak_idx[0]*self.config['ACQUISITION']['doppler_step'])
            self.code_doppler = self.carrier_doppler * self.kappa
            self.kf.UpdateDoppler(self.carrier_doppler)
            
            # initialize tracking buffer index
            self.buffer_ptr += (self.total_samples + first_peak_idx[1] - self.samples_per_ms)
            self.buffer_ptr %= self.rfbuffer.size
            
            # run initial tracking NCO
            self.total_samples = np.round(self.config['RFSIGNAL']['sampling_freq'] \
                    / ((GPS_L1CA_CODE_FREQ + self.code_doppler) / GPS_L1CA_CODE_SIZE))
            self.half_samples = int(self.total_samples / 2)
            self.samples_processed = 0
            
            # set channel mode to tracking
            self.channel_status.State = ChannelState.TRACKING
            self.channel_status.Doppler = self.carrier_doppler / NP_TWO_PI
            
            # TODO: print/log output
            self.logger.info(f'Channel {self.channel_status.ChannelNum}: ' \
                             f'{self.channel_status.ID} acquired! ' \
                             f'Doppler (Hz): {np.round(self.channel_status.Doppler).astype(np.int32)}, ' \
                             f'Code Phase (samples): {np.round(first_peak_idx[1]).astype(np.int32)}')
        else:
            self.channel_status.State = ChannelState.IDLE
            # print("Acquisition Failed", flush=True)
            
        return
    
# ------------------------------------------------------------------------------------------------ #

    def Track(self):
        """
        Runs the tracking correlation and loop filter updates!
        """
        # make sure an initial count has been made
        self.samples_remaining = self.rfbuffer.GetNumUnreadSamples(self.buffer_ptr)
        # print(f"samples_remaining = {self.samples_remaining}")
        
        while self.samples_remaining > 0:
            # recount samples remaining inside buffer
            # TODO: edge case of 0 samples remaining to start/end but more processing necessary
            self.samples_remaining = self.rfbuffer.GetNumUnreadSamples(self.buffer_ptr)
            
            # ----- INTEGRATE -----
            if self.samples_processed < self.total_samples:
                # determine number of samples to read
                samples_to_read = np.min(
                    [self.total_samples - self.samples_processed, self.samples_remaining]
                ).astype(np.uint32)
                
                # accumulate samples of current code period
                E, P, L, P_1, P_2, N, self.samples_processed, self.rem_carrier_phase, \
                        self.rem_code_phase = AccumulateEPL( \
                    self.rfbuffer.Pull(self.buffer_ptr, samples_to_read),
                    self.code,
                    self.tap_spacing,
                    self.config['RFSIGNAL']['sampling_freq'],
                    GPS_L1CA_CODE_SIZE,
                    GPS_L1CA_CODE_FREQ + self.code_doppler,
                    NP_TWO_PI * self.config['RFSIGNAL']['intermediate_freq'] + self.carrier_doppler,
                    self.carrier_jitter,
                    self.rem_code_phase,
                    self.rem_carrier_phase,
                    samples_to_read,
                    self.samples_processed,
                    self.half_samples,
                )
                self.IE   += E.real
                self.IP   += P.real
                self.IL   += L.real
                self.IN   += N.real
                self.IP_1 += P_1.real
                self.IP_2 += P_2.real
                self.QE   += E.imag
                self.QP   += P.imag
                self.QL   += L.imag
                self.QN   += N.imag
                self.QP_1 += P_1.imag
                self.QP_2 += P_2.imag
                
                # move forward in buffer
                self.buffer_ptr += samples_to_read
                self.buffer_ptr %= self.rfbuffer.size
                
            # ----- DUMP -----
            else:
                # print(f"samples_processed = {self.samples_processed}")
                samples_to_read = 0
                
                # check that data can be parsed and syncronize to the bits
                if not self.channel_status.Ephemeris:
                    if not self.channel_status.DataLock:
                        # check for bit flip
                        if self.NavDataSync():
                            continue
                                
                    # if data bits are syncronized, try demodulation
                    else:
                        self.Demodulate()
                        self.channel_status.ToW -= 0.02
                
                # discriminators
                phase_err = PllCostas(self.IP, self.QP)                                   # [rad]
                freq_err = FllAtan2(self.IP_1, self.IP_2, self.QP_1, self.QP_2, self.T/2) # [rad/s]
                if self.channel_status.CarrierLock:
                    chip_err = DllNcdp(self.IE, self.IP, self.IL)                         # [chip]
                else:
                    chip_err = DllNneml(self.IE, self.QE, self.IL, self.QL)               # [chip]
                
                # loop filter
                t = self.total_samples / self.config['RFSIGNAL']['sampling_freq']
                self.kf.UpdateIntegrationTime(t)
                x = self.kf.Run(phase_err, freq_err, chip_err)  
                self.rem_carrier_phase = np.remainder(x[0], NP_TWO_PI)
                self.carrier_doppler = x[1]
                self.carrier_jitter = x[2]
                self.rem_code_phase = np.mod(x[3], GPS_L1CA_CODE_SIZE)
                if self.rem_code_phase > GPS_L1CA_CODE_SIZE/2:
                    self.rem_code_phase -= GPS_L1CA_CODE_SIZE
                self.code_doppler = x[4] + self.kappa * (x[1] + self.T * x[2])
                self.kf.x[0] = self.rem_carrier_phase
                self.kf.x[3] = self.rem_code_phase
                
                # lock detectors
                # self.channel_status.CodeLock, self.cn0_mag, self.amp_memory, self.noise_memory = \
                #     CodeLockDetector(self.amp_memory, self.noise_memory, self.IP, self.QP, self.IN, \
                #                     self.QN, t)
                # self.channel_status.CarrierLock, self.IP_memory, self.QP_memory = PhaseLockDetector( \
                #     self.IP_memory, self.QP_memory, self.IP, self.QP)
                self.channel_status.CodeLock, self.channel_status.CarrierLock, self.cn0_mag, \
                    self.NBD_memory, self.NBP_memory, self.N0P_memory = \
                        CodeAndCarrierLockDetectors(self.NBD_memory, 
                                                    self.NBP_memory, 
                                                    self.N0P_memory, 
                                                    self.IP, 
                                                    self.QP, 
                                                    self.IN, 
                                                    self.QN, 
                                                    t)
                self.kf.UpdateCn0(self.cn0_mag)
                
                # update channel tracking status
                self.channel_status.Doppler = self.carrier_doppler / NP_TWO_PI
                self.channel_status.CNo = 10.0*np.log10(self.cn0_mag) if self.cn0_mag > 0.0 else 0.0
                self.channel_status.IP = self.IP
                self.channel_status.QP = self.QP
                self.channel_status.IE = self.IE
                self.channel_status.QE = self.QE
                self.channel_status.IL = self.IL
                self.channel_status.QL = self.QL
                self.channel_status.IN = self.IN
                self.channel_status.QN = self.QN
                self.channel_status.IP_1 = self.IP_1
                self.channel_status.QP_1 = self.QP_1
                self.channel_status.IP_2 = self.IP_2
                self.channel_status.QP_2 = self.QP_2
                self.channel_status.ToW += 0.02
                
                # begin next NCO period
                self.counter += self.PDI
                self.total_samples = np.round(self.PDI * self.config['RFSIGNAL']['sampling_freq'] \
                                    / ((GPS_L1CA_CODE_FREQ + self.code_doppler) / GPS_L1CA_CODE_SIZE)).astype(np.uint32)
                self.half_samples = np.round(self.total_samples / 2).astype(np.uint32)
                self.samples_processed = 0
                self.IE   = 0.0
                self.IP   = 0.0
                self.IL   = 0.0
                self.IN   = 0.0
                self.IP_1 = 0.0
                self.IP_2 = 0.0
                self.QE   = 0.0
                self.QP   = 0.0
                self.QL   = 0.0
                self.QN   = 0.0
                self.QP_1 = 0.0
                self.QP_2 = 0.0
            # print(f"samples_to_read = {samples_to_read}")

        self.channel_status.CodePhase = self.rem_code_phase 
        self.channel_status.CarrierPhase = self.rem_carrier_phase 
            
        # # update nav packet
        # # TODO: make navigation queue/event
        # if self.channel_status.Ephemeris:
        #     self.nav_packet.header.ChannelNum = self.channel_status.header.ChannelNum
        #     self.nav_packet.header.Constellation = self.channel_status.header.Constellation
        #     self.nav_packet.header.Signal = self.channel_status.header.Signal
        #     self.nav_packet.header.ID = self.channel_status.header.ID
        #     self.nav_packet.header.Week = self.channel_status.header.Week
        #     self.nav_packet.header.ToW = self.channel_status.header.ToW
        #     self.nav_packet.SampleCount = self.samples_since_tow
        #     self.nav_packet.Doppler = self.channel_status.Doppler
        #     self.nav_packet.CNo = self.channel_status.CNo
        #     clk,pos,vel,acc = self.lnav_parser.GetNavStates(self.nav_packet.header.ToW, False)
        #     self.nav_packet.SatPos = pos
        #     self.nav_packet.SatVel = vel
        #     self.nav_packet.ClkCorr = clk[0]
        return
    
# ------------------------------------------------------------------------------------------------ #

    def NavDataSync(self):
        """
        Synchronize to the data bit and extend the integration periods
        """
        # check for bit flip
        if np.sign(self.IP) != np.sign(self.channel_status.IP):
            self.bit_sync_hist[self.counter % 20] += 1
        
            # test for data lock
            if self.counter > self.min_converg_time:
                tmp = self.bit_sync_hist.argsort()
                ratio = self.bit_sync_hist[tmp[-1]] / self.bit_sync_hist[tmp[-2]]
                if ratio > 4.0:
                    self.channel_status.DataLock = True
                    # print(f"{self.channel_status.ID}: Hist = {self.bit_sync_hist}")
                    # print(f"{self.channel_status.ID}: Bit sync! i = {tmp[-1]}, ratio = {ratio}, ms_processed = {self.counter}")
                    
                    # immediately continue current integration period and set T to 20 ms
                    self.T = 0.02
                    self.PDI = 20
                    self.total_samples = np.round(self.PDI * self.config['RFSIGNAL']['sampling_freq'] \
                            / ((GPS_L1CA_CODE_FREQ + self.code_doppler) / GPS_L1CA_CODE_SIZE)).astype(np.uint32)
                    self.half_samples = np.round(self.total_samples / 2).astype(np.uint32)
                    
                    # reset filters
                    self.tap_spacing = self.config["TRACKING"]["correlator_epl_narrow"]
                    self.w0p = NaturalFrequency(self.config["TRACKING"]["pll_bandwidth"], 3)
                    self.w0f = NaturalFrequency(self.config["TRACKING"]["fll_bandwidth"], 2)
                    self.w0d = NaturalFrequency(self.config["TRACKING"]["dll_bandwidth"], 2)
                    self.kf.UpdateNaturalFreqs(self.w0p, self.w0f, self.w0d)
                    # self.kf.UpdateIntegrationTime(self.T)
                    # self.amp_memory   = 0.0
                    # self.noise_memory = 0.0
                    # self.IP_memory    = 0.0
                    # self.QP_memory    = 0.0
                    return True
        return False
    
# ------------------------------------------------------------------------------------------------ #

    def Demodulate(self):
        """
        Demodulate the navigation data and parse ephemerides.
        """
            
        # keep replaceing bits while phase locked
        if self.lnav_parser.NextBit(self.IP > 0):
            # reset sample counter after a subframe has been parsed
            self.samples_since_tow   = 0
            self.channel_status.Week = self.lnav_parser.week
            self.channel_status.ToW  = self.lnav_parser.TOW
    
        if self.lnav_parser.subframe1 and self.lnav_parser.subframe2 and self.lnav_parser.subframe3:
            self.channel_status.Ephemeris = True
            self.lnav_parser.SetID(self.channel_status.ID)
            
            # save ephemerids to csv file and log to terminal
            self.logger.debug(f"Channel {self.channel_status.ChannelNum} - {self.channel_status.ID} Ephemeris:\n"
                                f"\t\t\t\t  -----------------------------------\n"
                                f"\t\t\t\t  week     = {self.lnav_parser.week}\n"
                                f"\t\t\t\t  ura      = {self.lnav_parser.ephemerides.ura}\n"
                                f"\t\t\t\t  health   = {self.lnav_parser.ephemerides.health}\n"
                                f"\t\t\t\t  T_GD     = {self.lnav_parser.ephemerides.tgd}\n"
                                f"\t\t\t\t  IODC     = {self.lnav_parser.ephemerides.iodc}\n"
                                f"\t\t\t\t  t_oc     = {self.lnav_parser.ephemerides.toc}\n"
                                f"\t\t\t\t  af2      = {self.lnav_parser.ephemerides.af2}\n"
                                f"\t\t\t\t  af1      = {self.lnav_parser.ephemerides.af1}\n"
                                f"\t\t\t\t  af0      = {self.lnav_parser.ephemerides.af0}\n"
                                f"\t\t\t\t  IODE_SF2 = {self.lnav_parser.ephemerides.iode}\n"
                                f"\t\t\t\t  C_rs     = {self.lnav_parser.ephemerides.crs}\n"
                                f"\t\t\t\t  deltan   = {self.lnav_parser.ephemerides.deltan}\n"
                                f"\t\t\t\t  M_0      = {self.lnav_parser.ephemerides.m0}\n"
                                f"\t\t\t\t  C_uc     = {self.lnav_parser.ephemerides.cuc}\n"
                                f"\t\t\t\t  e        = {self.lnav_parser.ephemerides.e}\n"
                                f"\t\t\t\t  C_us     = {self.lnav_parser.ephemerides.cus}\n"
                                f"\t\t\t\t  sqrtA    = {self.lnav_parser.ephemerides.sqrtA}\n"
                                f"\t\t\t\t  t_oe     = {self.lnav_parser.ephemerides.toe}\n"
                                f"\t\t\t\t  C_ic     = {self.lnav_parser.ephemerides.cic}\n"
                                f"\t\t\t\t  omega0   = {self.lnav_parser.ephemerides.omega0}\n"
                                f"\t\t\t\t  C_is     = {self.lnav_parser.ephemerides.cis}\n"
                                f"\t\t\t\t  i_0      = {self.lnav_parser.ephemerides.i0}\n"
                                f"\t\t\t\t  C_rc     = {self.lnav_parser.ephemerides.crc}\n"
                                f"\t\t\t\t  omega    = {self.lnav_parser.ephemerides.omega}\n"
                                f"\t\t\t\t  omegaDot = {self.lnav_parser.ephemerides.omegaDot}\n"
                                f"\t\t\t\t  IODE_SF3 = {self.lnav_parser.ephemerides.iode}\n"
                                f"\t\t\t\t  iDot     = {self.lnav_parser.ephemerides.iDot}"
            )
            self.log_queue.put(self.lnav_parser.ephemerides)
                
# ------------------------------------------------------------------------------------------------ #

    @property
    def GetNavStates(self, transit_time: np.double, calc_accel: bool=False):
        clk, pos, vel, acc = \
            self.lnav_parser.GetNavStates(self.channel_status.ToW - transit_time, calc_accel)
        return clk, pos, vel, acc
    
# ===== GPS L1 C/A Code Generator ================================================================ #

delay = np.asarray(
    [
        [1,5],
        [2,6],
        [3,7],
        [4,8],
        [0,8],
        [1,9],
        [0,7],
        [1,8], 
        [2,9],
        [1,2],
        [2,3],
        [4,5],
        [5,6],
        [6,7],
        [7,8],
        [8,9],
        [0,3],
        [1,4],
        [2,5],
        [3,6],
        [4,7],
        [5,8],
        [0,2],
        [3,5],
        [4,6],
        [5,7],
        [6,8],
        [7,9],
        [0,5],
        [1,6],
        [2,7],
        [3,8],
    ],
    dtype=np.uint8,
)
# delay = {
#     1 : (1,5),
#     2 : (2,6),
#     3 : (3,7),
#     4 : (4,8),
#     5 : (0,8),
#     6 : (1,9),
#     7 : (0,7),
#     8 : (1,8), 
#     9 : (2,9),
#     10: (1,2),
#     11: (2,3),
#     12: (4,5),
#     13: (5,6),
#     14: (6,7),
#     15: (7,8),
#     16: (8,9),
#     17: (0,3),
#     18: (1,4),
#     19: (2,5),
#     20: (3,6),
#     21: (4,7),
#     22: (5,8),
#     23: (0,2),
#     24: (3,5),
#     25: (4,6),
#     26: (5,7),
#     27: (6,8),
#     28: (7,9),
#     29: (0,5),
#     30: (1,6),
#     31: (2,7),
#     32: (3,8),
# }

@njit(cache=True, fastmath=True)
def gps_l1ca_code(prn: np.uint8):
    """Generates the GPS L1 C/A code based on the requested PRN

    Parameters
    ----------
    prn : np.uint8
        Satellite ID

    Returns
    -------
    x : np.ndarray[np.int8]
        Requested C/A code
    """
    # initialize registers
    g1 = np.ones(10)
    g2 = np.ones(10)
    s1 = delay[prn-1][0]
    s2 = delay[prn-1][1]
    
    x = np.zeros(1023, dtype=np.int8)
    for i in range(1023):
        # next code chip
        g2i = (((g2[s1] + g2[s2]) % 2) + g1[9]) % 2
        x[i] = 2 * g2i - 1 # enforce +/- 1
        
        # apply register feedback
        f1 = (g1[9] + g1[2]) % 2
        f2 = (g2[9] + g2[8] + g2[7] + g2[5] + g2[2] + g2[1]) % 2
        for j in range(9,0,-1):
            g1[j] = g1[j-1]
            g2[j] = g2[j-1]
        g1[0] = f1
        g2[0] = f2
    
    return x