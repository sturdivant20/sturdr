"""**channel_controller.py**

======  ============================================================================================
file    sturdr/rcvr/rf_data_buffer.py 
brief   Handles channel processing and dissemination of data to channels
date    October 2024
======  ============================================================================================
"""

import logging.handlers
import multiprocessing.synchronize
import numpy as np
import logging
import time
import multiprocessing
from multiprocessing import Process, Queue, Barrier
from sturdr.utils.rf_data_buffer import RfDataBuffer
from sturdr.utils.enums import GnssSignalTypes
from sturdr.channel import channel, gps_l1ca_channel

# TODO: add more channel types in "SpawnChannels"

class ChannelController(Process):
    """
    Maintains the status of each channel and ensures they are synchronized to the 
    :obj:`multiprocessing.shared_memory.SharedMemory` circular buffer of RF signal data.
    """
    
    __slots__ = 'config', 'rfbuffer', 'memory', 'log_queue', 'logger', 'channels', 'nchannels'
    config        : dict                                # Receiver config
    rfbuffer      : RfDataBuffer                        # file parser and memory manager
    log_queue     : Queue                               # queue/pipe for logging
    logger        : logging.Logger                      # thread safe logger
    start_barrier : multiprocessing.synchronize.Barrier # Syncronizes the start of data proccessing from shared memory
    done_barrier  : multiprocessing.synchronize.Barrier # Indicated end of data proccessing and reads next chunk of data
    channels      : list[channel.Channel]               # contains status/id information about channels
    nchannels     : int                                 # number of active channels
    
    def __init__(self, config: dict, log_queue: Queue):
        Process.__init__(self, name='SturDR_ChannelManager')
        self.config = config
        
        # initialize dict of channels
        # self.channels = {
        #     "GPS"     : [],
        #     "GALILEO" : [],
        #     "GLONASS" : [],
        #     "BEIDOU"  : [],
        #     "QZSS"    : [],
        #     "IRNSS"   : [],
        #     "SBAS"    : [],
        # }
        self.channels = []
        self.nchannels = sum(self.config['CHANNELS']['max_channels'])
        
        # find and initialize logger
        self.logger = logging.getLogger('SturDR_Logger')
        self.log_queue = log_queue
        
        # initialize RfDataBuffer, shared memory, and memory queue
        self.rfbuffer = RfDataBuffer(config)
        self.start_barrier = Barrier(self.nchannels + 1)
        self.done_barrier = Barrier(self.nchannels + 1)
    
    def SpawnChannels(self, signal_type: list[str]|np.ndarray[str], nchan: list[int]|np.ndarray[int]):
        """
        Spawns the requested number of channels with the requested type
        """
        channel_count = 0
        for s,n in zip(signal_type, nchan):
            for i in range(n):
                # GPS L1CA Channel
                if s.casefold() == 'gps_l1ca':
                    channel_id = f'SturDR_Ch{channel_count}_{str(GnssSignalTypes.GPS_L1CA)}'
                    self.channels.append(gps_l1ca_channel.GpsL1caChannel(
                            config=self.config,
                            cid=channel_id,
                            rfbuffer=self.rfbuffer,
                            log_queue=self.log_queue,
                            start_barrier=self.start_barrier,
                            done_barrier=self.done_barrier,
                            num=i
                        )
                    )
                
                # log channel spawning
                self.logger.debug(f"Channel Controller: {channel_id} spawned.")
                channel_count += 1
        return
    
    def run(self):
        start_t = time.time()
        
        # spawn the channels
        self.SpawnChannels(self.config['CHANNELS']['signals'], self.config['CHANNELS']['max_channels'])
        self.channels[0].SetSatellite(1)
        self.channels[0].start()
        self.channels[1].SetSatellite(7)
        self.channels[1].start()
        self.channels[2].SetSatellite(14)
        self.channels[2].start()
        self.channels[3].SetSatellite(17)
        self.channels[3].start()
        self.channels[4].SetSatellite(19)
        self.channels[4].start()
        self.channels[5].SetSatellite(21)
        self.channels[5].start()
        self.channels[6].SetSatellite(30)
        self.channels[6].start()
        
        ms_processed = 0
        while ms_processed < self.config['GENERAL']['ms_to_process']:
            # read new data and inform channel
            self.rfbuffer.NextChunk()
            self.start_barrier.wait()
            
            # wait for the channels and process new data
            self.done_barrier.wait()
            
            # increment total time processed
            ms_processed += self.config['GENERAL']['ms_read_size']
        
        end_t = time.time()
        self.logger.info(f"Total Time = {1000 * (end_t - start_t)} ms")
        self.log_queue.put(None)
        return
