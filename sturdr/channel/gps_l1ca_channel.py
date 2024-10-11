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
from multiprocessing import shared_memory, Queue, Event

from sturdr.channel.channel import Channel, ChannelStatus
from sturdr.dsp.acquisition import PcpsSearch, Peak2NoiseFloorComparison
from sturdr.dsp.tracking import NaturalFrequency, TrackingKF
from sturdr.dsp.gnss_signal import CodeNCO, CarrierNCO, CorrelateEPL, Correlate
from sturdr.dsp.discriminator import PllCostas, DllNneml, FllAtan2
from sturdr.dsp.lock_detector import CodeLockDetector, PhaseLockDetector
from sturdr.nav.gps_lnav import GpsLnavParser
from sturdr.utils.constants import GPS_L1CA_CARRIER_FREQ, GPS_L1CA_CODE_FREQ, GPS_L1CA_CODE_SIZE
from sturdr.utils.enums import ChannelState, GnssSystem, GnssSignalTypes
from sturdr.utils.rf_data_buffer import RfDataBuffer

NP_TWO_PI = 2 * np.pi

class GpsL1caChannel(Channel):
    __slots__ = 'code', 'code_replica', 'rem_code_phase', 'code_doppler', 'carrier_replica', \
                'rem_carrier_phase', 'carrier_doppler', 'carrier_jitter', 'T', 'total_samples', \
                'half_samples', 'kf', 'tap_spacing', 'IP', 'QP', 'IE', 'QE', 'IL', 'QL', 'IP_1', \
                'QP_1', 'IP_2', 'QP_2', 'IN', 'QN', 'cn0_mag', 'amp_memory', 'noise_memory', \
                'IP_memory', 'QP_memory', 'bit_sync', 'preamble_found', 'samples_since_tow', \
                'samples_per_ms', 'samples_per_chip'
    
    # Replicas
    code              : np.ndarray[np.int8]     # PRN replica code
    code_replica      : np.ndarray[np.double]   # upsampled code replica
    rem_code_phase    : np.double               # remainder code phase
    code_doppler      : np.double               # current code doppler
    carrier_replica   : np.ndarray[np.double]   # upsampled carrier replica
    rem_carrier_phase : np.double               # remainder carrier phase
    carrier_doppler   : np.double               # current carrier doppler
    carrier_jitter    : np.double               # current carrier doppler rate
    T                 : np.double               # current integration time [s]
    total_samples     : int                     # predicted number of total samples in integration period
    half_samples      : int                     # half of total_samples
    
    # Tracking
    w0p               : np.double
    w0d               : np.double
    w0f               : np.double
    kf                : TrackingKF          # code/carrier tracking filter
    tap_spacing       : int                 # spacing between early, promt, and late correlators
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
    cn0_mag           : np.double
    amp_memory        : np.double
    noise_memory      : np.double
    IP_memory         : np.double
    QP_memory         : np.double

    # Telemetry
    bit_sync          : bool                # has channel been syncronized to bits?
    preamble_found    : bool                # has preamble been detected
    samples_since_tow : int                 # number of samples since last TOW (preamble) was parsed
    lnav_parser       : GpsLnavParser
    
    # Sample constants
    samples_per_ms    : int                 # samples in 1 millisecond of RF data
    samples_per_chip  : int                 # samples in 1 chip of the code
    
# ------------------------------------------------------------------------------------------------ #

    def __init__(self, config: dict, cid: str, rfbuffer: RfDataBuffer, queue: Queue, num: int):
        Channel.__init__(self, config, cid, rfbuffer, queue, num)
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
        
        # tracking
        self.tap_spacing = int(self.config['TRACKING']['correlator_epl_wide'] * self.samples_per_chip)
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
        self.amp_memory   = 0.0
        self.noise_memory = 0.0
        self.IP_memory    = 0.0
        self.QP_memory    = 0.0
        
        # telemetry
        self.bit_sync       = False
        self.preamble_found = False
        
        return

# ------------------------------------------------------------------------------------------------ #

    def run(self):
        # print("Run", flush=True)
        while True:
            # wait for new data to process
            self.event_start.wait()
            self.event_start.clear()
            
            # update the write pointer location for this process
            #   -> necessary because `rfbuffer` is in `another Process` which cannot be seen in 
            #      `this Process`
            self.rfbuffer.UpdateWritePtr(self.samples_per_ms)
            
            # process data
            if self.channel_status.State == ChannelState.TRACKING:
                self.Track()
            elif self.channel_status.State == ChannelState.ACQUIRING:
                self.Acquire()
            
            # send the signal that the channel is finished
            self.queue.put(self.channel_status)
            self.event_done.set()
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
        self.total_samples  = int(self.config['ACQUISITION']['coherent_integration'] * \
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
        
        # upsample code
        self.code_replica, _ = CodeNCO(self.code, 
                                       self.config['RFSIGNAL']['sampling_freq'], 
                                       GPS_L1CA_CODE_FREQ, 
                                       GPS_L1CA_CODE_SIZE
                                )
        
        # run acquisition
        correlation_map = PcpsSearch(self.rfbuffer.Pull(self.buffer_ptr, self.total_samples), 
                                     self.code_replica,
                                     self.config['ACQUISITION']['doppler_range'],
                                     self.config['ACQUISITION']['doppler_step'],
                                     self.config['RFSIGNAL']['sampling_freq'],
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
            self.buffer_ptr += (self.total_samples - self.samples_per_ms + first_peak_idx[1])
            self.buffer_ptr %= self.rfbuffer.size
            
            # run initial tracking NCO
            self.NCO()
            
            # set channel mode to tracking
            self.channel_status.State = ChannelState.TRACKING
            self.channel_status.Doppler = self.carrier_doppler / NP_TWO_PI
            
            # TODO: print/log output
            # print("Acquisition Success", flush=True)
        else:
            self.channel_status.State = ChannelState.IDLE
            # print("Acquisition Failed", flush=True)
            
        return
    
# ------------------------------------------------------------------------------------------------ #

    def Track(self):
        """
        Runs the tracking correlation and loop filter updates!
        """
        # print("Track", flush=True)
        
        # make sure enough samples have been parsed
        if self.rfbuffer.GetNumUnreadSamples(self.buffer_ptr) < self.total_samples:
            return
        
        # correlate
        signal = self.rfbuffer.Pull(self.buffer_ptr, self.total_samples) * self.carrier_replica
        self.IE, self.QE, self.IP, self.QP, self.IL, self.QL, self.IP_1, self.QP_1, self.IP_2, \
            self.QP_2, self.IN, self.QN = CorrelateEPL(signal, 
                                                       self.code_replica,
                                                       self.tap_spacing,
                                                       True)
        
        # discriminators
        phase_err = PllCostas(self.IP, self.QP)                                   # [rad]
        freq_err = FllAtan2(self.IP_1, self.IP_2, self.QP_1, self.QP_2, self.T/2) # [rad/s]
        chip_err = DllNneml(self.IE, self.QE, self.IL, self.QL)                   # [chip]
        
        # loop filter
        x = self.kf.Run(phase_err, freq_err, chip_err)  
        self.rem_carrier_phase = np.remainder(x[0], NP_TWO_PI)
        self.carrier_doppler = x[1]
        self.carrier_jitter = x[2]
        self.rem_code_phase = np.mod(x[3], GPS_L1CA_CODE_SIZE)
        self.code_doppler = x[4] + self.kappa * (x[1] + self.T * x[2])
        
        # lock detectors
        self.channel_status.CodeLock, self.cn0_mag, self.amp_memory, self.noise_memory = \
            CodeLockDetector(self.amp_memory, self.noise_memory, self.IP, self.QP, self.IN, \
                             self.QN, self.T)
        self.channel_status.CarrierLock, self.IP_memory, self.QP_memory = PhaseLockDetector( \
            self.IP_memory, self.QP_memory, self.IP, self.QP)
        
        # move forward in buffer
        self.buffer_ptr += self.total_samples
        self.buffer_ptr %= self.rfbuffer.size
        
        # update channel status
        self.channel_status.Doppler = self.carrier_doppler / NP_TWO_PI
        self.channel_status.IP = self.IP
        self.channel_status.QP = self.QP
        self.channel_status.CN0 = 10.0*np.log10(self.cn0_mag) if self.cn0_mag > 0.0 else 0.0
        
        # run next NCO
        self.NCO()
        return
    
# ------------------------------------------------------------------------------------------------ #

    def NCO(self):
        """
        Runs the carrier and code numerically controlled oscillators
        """
        # run next NCO
        self.code_replica, _ = CodeNCO(self.code, 
                                       self.config['RFSIGNAL']['sampling_freq'], 
                                       GPS_L1CA_CODE_FREQ + self.code_doppler, 
                                       GPS_L1CA_CODE_SIZE, 
                                       self.rem_code_phase)
        self.total_samples = self.code_replica.size
        self.half_samples = int(self.total_samples / 2)
        self.carrier_replica, _ = CarrierNCO(self.config['RFSIGNAL']['sampling_freq'],
                                             NP_TWO_PI * self.config['RFSIGNAL']['intermediate_freq'] \
                                                + self.carrier_doppler,
                                             self.carrier_jitter,
                                             self.total_samples,
                                             self.rem_carrier_phase)
        return
    
# ------------------------------------------------------------------------------------------------ #

    def Decode(self):
        pass
    
# ===== GPS L1 C/A Code Generator ================================================================ #

delay = {
    1 : (1,5),
    2 : (2,6),
    3 : (3,7),
    4 : (4,8),
    5 : (0,8),
    6 : (1,9),
    7 : (0,7),
    8 : (1,8), 
    9 : (2,9),
    10: (1,2),
    11: (2,3),
    12: (4,5),
    13: (5,6),
    14: (6,7),
    15: (7,8),
    16: (8,9),
    17: (0,3),
    18: (1,4),
    19: (2,5),
    20: (3,6),
    21: (4,7),
    22: (5,8),
    23: (0,2),
    24: (3,5),
    25: (4,6),
    26: (5,7),
    27: (6,8),
    28: (7,9),
    29: (0,5),
    30: (1,6),
    31: (2,7),
    32: (3,8),
}

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
    s1 = delay[prn][0]
    s2 = delay[prn][1]
    
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