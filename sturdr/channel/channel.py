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

import logging.handlers
import numpy as np
import logging
import multiprocessing
from multiprocessing.synchronize import Barrier
from multiprocessing import Process, Queue, shared_memory
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from sturdr.rcvr.rf_data_buffer import RfDataBuffer
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
    IN            : np.double       = np.nan
    QN            : np.double       = np.nan
    IP_1          : np.double       = np.nan
    QP_1          : np.double       = np.nan
    IP_2          : np.double       = np.nan
    QP_2          : np.double       = np.nan
    
@dataclass(order=True, slots=True)
class NavPacket:
    """
    Minimal packet of navigation data to be sent to the Navigator
    """
    ChannelNum    : np.uint8        = -1
    Signal        : GnssSignalTypes = GnssSignalTypes.UNKNOWN
    ID            : str             = ''
    Week          : np.uint16       = np.nan
    ToW           : np.double       = np.nan
    CNo           : np.double       = np.nan
    Doppler       : np.double       = np.nan
    CodePhase     : np.double       = np.nan
    CarrierPhase  : np.double       = np.nan
    
    @staticmethod
    def dict_factory(x):
        exclude_fields = ("ChannelNum", "Signal", "ID")
        return {k: v for (k, v) in x if ((v is not None) and (k not in exclude_fields))}

class Channel(ABC, Process):
    """
    Abstract class for Channel object definitions.
    """
    __slots__ = 'config', 'channelID', 'channel_status', 'nav_status', 'rfbuffer', 'buffer_ptr', \
                'log_queue', 'nav_queue', 'start_barrier', 'end_barrier'
    config            : dict                                # dict of receiver config
    channelID         : str                                 # channel id
    channel_status    : ChannelPacket                       # Status to be shared across threads/processes
    nav_status        : ChannelPacket                       # Status to be shared to navigator
    rfbuffer          : RfDataBuffer                        # file parser and memory manager
    buffer_ptr        : int                                 # pointer to current index in rfbuffer data
    log_queue         : Queue                               # queue/pipe for channels finishing current timestep
    nav_queue         : Queue                               # queue/pipe for navigation updates
    start_barrier     : Barrier # barrier for synchronzing when new data is available
    done_barrier      : Barrier # barrier for synchronzing when new data has been processed
    logger            : logging.Logger                      # thread safe logger
    
    def __init__(self, 
                 config: dict, 
                 cid: str, 
                 rfbuffer: RfDataBuffer, 
                 log_queue: Queue,
                 nav_queue: Queue,
                 start_barrier: Barrier, 
                 done_barrier: Barrier, 
                 num: int):
        Process.__init__(self, name=cid, daemon=True)
        
        self.config                     = config
        self.channelID                  = cid
        self.rfbuffer                   = rfbuffer
        self.buffer_ptr                 = 0
        self.log_queue                  = log_queue
        self.nav_queue                  = nav_queue
        self.channel_status             = ChannelPacket()
        self.channel_status.ChannelNum  = num
        self.nav_status                 = NavPacket()
        self.nav_status.ChannelNum      = num
        self.start_barrier              = start_barrier
        self.done_barrier               = done_barrier
        
        # find and initialize logger
        self.logger = logging.getLogger('SturDR_Logger')
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