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
from abc import ABC, abstractmethod
from dataclasses import dataclass
from multiprocessing import Process, shared_memory, Queue, Event
from sturdr.utils.rf_data_buffer import RfDataBuffer
from sturdr.utils.enums import GnssSystem, GnssSignalTypes, ChannelState

@dataclass(order=True, slots=True)
class ChannelStatus:
    ChannelNum    : int             = -1
    Constellation : GnssSystem      = 0
    Signal        : GnssSignalTypes = 0
    ID            : str             = ''
    week          : int             = np.nan
    TOW           : np.double       = np.nan
    State         : ChannelState    = 0
    CodeLock      : bool            = False
    CarrierLock   : bool            = False
    DataLock      : bool            = False
    Ephemeris     : bool            = False
    Doppler       : np.double       = np.nan
    CN0           : np.double       = np.nan
    IP            : np.double       = np.nan
    QP            : np.double       = np.nan

class Channel(ABC, Process):
    """
    Abstract class for Channel object definitions.
    """
    __slots__ = 'config', 'channelID', 'channel_status', 'rfbuffer', 'buffer_ptr', 'queue', \
                'event_start', 'event_done'
    config            : dict            # dict of receiver config
    channelID         : str             # channel id
    channel_status    : ChannelStatus   # Status to be shared across threads/processes
    rfbuffer          : RfDataBuffer    # file parser and memory manager
    buffer_ptr        : int             # pointer to current index in rfbuffer data
    queue             : Queue           # queue/pipe for channels finishing current timestep
    event_start       : Event
    event_done        : Event
    
    
    def __init__(self, config: dict, cid: str, rfbuffer: RfDataBuffer, queue: Queue, num: int):
        Process.__init__(self, name=cid, daemon=True)
        
        self.config             = config
        self.channelID          = cid
        self.rfbuffer           = rfbuffer
        self.buffer_ptr         = 0
        self.queue              = queue
        self.event_start        = Event()
        self.event_done         = Event()
        self.channel_status     = ChannelStatus()
        self.channel_status.ChannelNum = num
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
    def NCO(self):
        """
        Runs the carrier and code numerically controlled oscillators
        """
        pass
    
    @abstractmethod
    def Decode(self):
        pass