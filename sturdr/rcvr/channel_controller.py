"""**channel_controller.py**

======  ============================================================================================
file    sturdr/rcvr/rf_data_buffer.py 
brief   Handles channel processing and dissemination of data to channels
date    October 2024
======  ============================================================================================
"""

import numpy as np
from multiprocessing import Process, shared_memory, Queue
from sturdr.utils.rf_data_buffer import RfDataBuffer
from sturdr.utils.enums import GnssSignalTypes
from sturdr.channel import gps_l1ca_channel

class ChannelController(Process):
    """
    Maintains the status of each channel and ensures they are synchronized to the 
    :obj:`multiprocessing.shared_memory.SharedMemory` circular buffer of RF signal data.
    """
    
    __slots__ = 'config', 'rfbuffer', 'memory', 'queue'
    config    : dict                        # Receiver config
    rfbuffer  : RfDataBuffer                # file parser and memory manager
    queue     : Queue                       # queue/pipe for channels finishing current timestep
    channels  : list                        # contains status/id information about channels
    nchannels : int                         # number of active channels
    
    def __init__(self, config: dict):
        Process.__init__(self, name='SturDR_ChannelManager')
        self.config = config
        
        # # initialize dict of channels
        # self.channels = {
        #     "GPS"     : [],
        #     "GALILEO" : [],
        #     "GLONASS" : [],
        #     "BEIDOU"  : [],
        #     "QZSS"    : [],
        #     "IRNSS"   : [],
        #     "SBAS"    : [],
        # }
        
        # initialize RfDataBuffer, shared memory, and memory queue
        self.rfbuffer = RfDataBuffer(config)
        self.queue  = Queue()
    
    def SpawnChannels(self, 
                      signal_type: GnssSignalTypes|np.ndarray[GnssSignalTypes], 
                      nchannels: int):
        for i in nchannels:
            channel_id = f'SturDR_Ch{self.nchannels}_{str(signal_type)}_SV{i}'
            
            # GPS L1CA Channel
            if signal_type == GnssSignalTypes.GPS_L1CA:
                self.channels.append(gps_l1ca_channel.GpsL1caChannel(
                        config=self.config,
                        cid=channel_id,
                        rfbuffer=self.rfbuffer,
                        queue=self.queue,
                    )
                )
            # TODO: add more channel types
            
            self.nchannels += 1
        return
