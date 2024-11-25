"""**channel.py**

======  ============================================================================================
file    sturdr/channel/channel.py
brief   Abstract class for GNSS channel definitions.
date    October 2024
refs    1. "Understanding GPS/GNSS Principles and Applications", 3rd Edition, 2017 
            - Kaplan & Hegarty
        2. "A Software-Defined GPS and Galileo Receiver: A Single-Frequency Approach", 2007
            - Borre, Akos, Bertelsen, Rinder, Jensen
======  ============================================================================================
"""

import numpy as np
import logging
import logging.handlers
from multiprocessing.synchronize import Barrier, Event
from multiprocessing.connection import Connection
from multiprocessing import Process, Queue, shared_memory
from threading import Thread
from abc import ABC, abstractmethod
from dataclasses import dataclass, field

# from sturdr.rcvr.logger import ColorFormatter
from sturdr.utils.enums import GnssSystem, GnssSignalTypes, ChannelState

@dataclass(order=True, slots=True)
class ChannelPacket:
    """
    Packet of data saved to the log file
    """
    ChannelNum    : np.uint8        = -1
    Constellation : GnssSystem      = GnssSystem.UNKNOWN
    Signal        : GnssSignalTypes = GnssSignalTypes.UNKNOWN
    ID            : str             = ''
    State         : ChannelState    = ChannelState.OFF
    CodeLock      : bool            = False
    CarrierLock   : bool            = False
    DataLock      : bool            = False
    Ephemeris     : bool            = False
    Week          : np.uint16       = np.nan
    ToW           : np.double       = np.nan
    CNo           : np.double       = np.nan
    Doppler       : np.double       = np.nan
    CodePhase     : np.double       = np.nan
    CarrierPhase  : np.double       = np.nan
    IP            : np.double       = np.nan
    QP            : np.double       = np.nan
    IE            : np.double       = np.nan
    QE            : np.double       = np.nan
    IL            : np.double       = np.nan
    QL            : np.double       = np.nan
    IP_1          : np.double       = np.nan
    QP_1          : np.double       = np.nan
    IP_2          : np.double       = np.nan
    QP_2          : np.double       = np.nan
    
@dataclass(order=True, slots=True)
class NavPacket:
    """
    Minimal packet of navigation data to be sent to the Navigator
    """
    ChannelNum    : np.int8         = -1
    Signal        : GnssSignalTypes = GnssSignalTypes.UNKNOWN
    ID            : str             = ''
    Week          : np.int16        = -1
    ToW           : np.double       = np.nan
    CNo           : np.double       = np.nan
    Doppler       : np.double       = np.nan
    CodePhase     : np.double       = np.nan
    ChipDisc      : np.double       = np.nan
    FreqDisc      : np.double       = np.nan
    
    @staticmethod
    def dict_factory(x):
        exclude_fields = ("ChannelNum", "Signal", "ID")
        return {k: v for (k, v) in x if ((v is not None) and (k not in exclude_fields))}
    
@dataclass(order=True, slots=True)
class VectorNcoUpdate:
    CodeDoppler    : np.double
    CarrierDoppler : np.double

class Channel(ABC, Process):
    """
    Abstract class for Channel object definitions.
    """
    __slots__ = 'config', 'channelID', 'channel_status', 'nav_status', 'buffer_ptr', 'buffer_len', \
                'buffer_read_len', 'log_queue', 'nav_queue', 'start_barrier', 'end_barrier', \
                'logger', 'shm', 'shm_array', 'vt_pipe', 'vt_thread', 'is_vt', 'vt_queue'
    config            : dict                                # dict of receiver config
    channelID         : str                                 # channel id
    channel_status    : ChannelPacket                       # Status to be shared across threads/processes
    nav_status        : NavPacket                           # Status to be shared to navigator
    buffer_ptr        : int                                 # pointer to current index of rfbuffer writer
    buffer_ptr        : int                                 # pointer to current index in rfbuffer data
    buffer_len        : int                                 # total rfbuffer size
    buffer_read_len   : int                                 # length of each buffer read
    log_queue         : Queue                               # queue/pipe for channels finishing current timestep
    nav_queue         : Queue                               # queue/pipe for navigation updates
    start_barrier     : Barrier # barrier for synchronzing when new data is available
    done_barrier      : Barrier # barrier for synchronzing when new data has been processed
    logger            : logging.Logger                      # thread safe logger
    shm               : shared_memory.SharedMemory
    shm_array         : np.ndarray
    vt_pipe           : Connection
    vt_thread         : Thread
    is_vt             : bool
    vt_queue          : Queue
    
    def __init__(self, 
                 config: dict, 
                 cid: str, 
                 log_queue: Queue,
                 nav_queue: Queue,
                 vt_queue: Queue,
                 start_barrier: Barrier, 
                 done_barrier: Barrier, 
                 vt_pipe: Connection,
                 num: int):
        Process.__init__(self, name=cid, daemon=True)
        
        self.config                     = config
        self.channelID                  = cid
        self.writer_ptr                 = 0
        self.buffer_ptr                 = 0
        self.buffer_len                 = int(config['GENERAL']['ms_chunk_size'] * config['RFSIGNAL']['sampling_freq'] / 1000)
        self.buffer_read_len            = int(config['GENERAL']['ms_read_size'] * config['RFSIGNAL']['sampling_freq'] / 1000)
        self.log_queue                  = log_queue
        self.nav_queue                  = nav_queue
        self.channel_status             = ChannelPacket()
        self.channel_status.ChannelNum  = num
        self.nav_status                 = NavPacket()
        self.nav_status.ChannelNum      = num
        self.start_barrier              = start_barrier
        self.done_barrier               = done_barrier
        self.vt_pipe                    = vt_pipe
        self.is_vt                      = False
        self.vt_queue                   = vt_queue

        return
    
    def __del__(self):
        # kill vt_thread
        # if self.vt_thread.is_alive():
        #     self.vt_thread.join()
        return
    
    @abstractmethod
    def run(self):
        pass
    
    @abstractmethod
    def SetSatellite(self, satelliteID: np.uint8):
        """
        Set the GNSS signal and satellite tracked by the channel.
        """
        return
    
    @abstractmethod
    def Acquire(self):
        """
        Trys to acquire the set satellite
        """
        pass
    
    @abstractmethod
    def Track(self):
        """
        Runs the tracking correlation and loop filter updates!
        """
        pass
    
    @abstractmethod
    def NavDataSync(self):
        """
        Synchronize to the data bit and extend the integration periods
        """
        pass
    
    @abstractmethod
    def Demodulate(self):
        """
        Demodulate the navigation data and parse ephemerides.
        """
        pass

    def InitMultiProcessing(self):
        """
        These items need to be initialized after this process has started
        """
        # find and initialize logger
        self.logger = logging.getLogger('SturDR_Logger')
        self.logger.setLevel(self.config['GENERAL']['log_level'])
        self.logger.addHandler(logging.handlers.QueueHandler(self.log_queue))

        # intialize shared memory
        if self.config['RFSIGNAL']['is_complex'] == 'true':
            dtype = np.dtype(np.complex128)
        else:
            dtype = np.dtype(np.float64)
        self.shm = shared_memory.SharedMemory(name='SturDR_RFData', create=False)
        self.shm_array = np.ndarray(self.buffer_len, dtype=dtype, buffer=self.shm.buf)
        
        # Vector processing thread
        self.vt_thread = Thread(target=self.WaitForVectorMode)
        self.vt_thread.start()
        return
    
    def WaitForVectorMode(self):
        """
        Separate thread that waits for the initialization of vector processing. Vector processing 
        will begin at the end of the current integration period.
        """
        self.is_vt = self.vt_pipe.recv()
        self.logger.critical(
            f"Channel {self.channel_status.ChannelNum} ({self.channelID}) "
            f"{('is not', 'is')[self.is_vt]} starting vector processing"
        )
        return

    def UpdateWriterPtr(self):
        self.writer_ptr += self.buffer_read_len
        self.writer_ptr %= self.buffer_len
        return
    
    def GetNumUnreadSamples(self):
        if self.buffer_ptr <= self.writer_ptr:
            return self.writer_ptr - self.buffer_ptr
        else:
            return self.buffer_len - self.buffer_ptr + self.writer_ptr