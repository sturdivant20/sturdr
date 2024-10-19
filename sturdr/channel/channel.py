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
from multiprocessing import Process, Queue #, Barrier
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from sturdr.utils.rf_data_buffer import RfDataBuffer
from sturdr.utils.enums import GnssSystem, GnssSignalTypes, ChannelState

@dataclass(order=True, slots=True)
class ChannelPacket:
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
    SampleCount   : np.uint64       = np.nan
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

class Channel(ABC, Process):
    """
    Abstract class for Channel object definitions.
    """
    __slots__ = 'config', 'channelID', 'channel_status', 'rfbuffer', 'buffer_ptr', \
                'log_queue', 'start_barrier', 'end_barrier'
    config            : dict                                # dict of receiver config
    channelID         : str                                 # channel id
    channel_status    : ChannelPacket                       # Status to be shared across threads/processes
    rfbuffer          : RfDataBuffer                        # file parser and memory manager
    buffer_ptr        : int                                 # pointer to current index in rfbuffer data
    log_queue         : Queue                               # queue/pipe for channels finishing current timestep
    start_barrier     : Barrier # barrier for synchronzing when new data is available
    done_barrier      : Barrier # barrier for synchronzing when new data has been processed
    logger            : logging.Logger                      # thread safe logger
    
    def __init__(self, 
                 config: dict, 
                 cid: str, 
                 rfbuffer: RfDataBuffer, 
                 log_queue: Queue,
                 start_barrier: Barrier, 
                 done_barrier: Barrier, 
                 num: int):
        Process.__init__(self, name=cid, daemon=True)
        
        self.config                     = config
        self.channelID                  = cid
        self.rfbuffer                   = rfbuffer
        self.buffer_ptr                 = 0
        self.log_queue                  = log_queue
        self.channel_status             = ChannelPacket()
        self.channel_status.ChannelNum  = num
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