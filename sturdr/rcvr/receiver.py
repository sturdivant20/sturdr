"""**receiver.py**

======  ============================================================================================
file    sturdr/rcvr/receiver.py 
brief   Handles all receiver operations: channels, navigation, logging
date    November 2024
refs    1. "Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems", 2nd Edition, 2013
            - Groves
        2. "A Software-Defined GPS and Galileo Receiver: A Single-Frequency Approach", 2007
            - Borre, Akos, Bertelsen, Rinder, Jensen
        3. "Global Positioning System: Signals, Measurements, and Performance", 2nd Edition, 2006
            - Misra & Enge
======  ============================================================================================
"""

import time
import yaml
import logging
import logging.handlers
import multiprocessing.synchronize
from multiprocessing import Queue, Barrier
from pathlib import Path
import numpy as np

from sturdr.rcvr.navigator import Navigator
from sturdr.rcvr.logger import Logger
from sturdr.rcvr.rf_data_buffer import RfDataBuffer
from sturdr.channel import channel, gps_l1ca_channel
from sturdr.utils.enums import GnssSignalTypes

class Reciever:
    
    config        : dict
    rfbuffer      : RfDataBuffer
    start_t       : float
    log_update_ms : int
    nav_update_ms : int
    log           : logging.Logger
    logger        : Logger
    log_queue     : Queue
    start_barrier : multiprocessing.synchronize.Barrier
    done_barrier  : multiprocessing.synchronize.Barrier
    channels      : list[channel.Channel]
    num_channels  : int
    navigator     : Navigator
    nav_queue     : Queue
    
    def __init__(self, config_file: Path | str):
        # Load Configuration
        with open(config_file, 'r') as file:
            self.config = yaml.safe_load(file)
            
        # open rf data buffer
        self.rfbuffer = RfDataBuffer(self.config)
        
        # Initialize channel parameters
        self.channels = []
        self.num_channels = sum(self.config['CHANNELS']['max_channels'])
        self.start_barrier = Barrier(self.num_channels + 1)
        self.done_barrier = Barrier(self.num_channels + 1)
            
        # Create Logging Process and obtain copy
        self.log_queue = Queue()
        self.logger = Logger(self.config, self.log_queue)
        self.logger.start()
        self.log = logging.getLogger('SturDR_Logger')
        self.log.addHandler(logging.handlers.QueueHandler(self.log_queue))
        self.log.setLevel(logging.DEBUG)
        
        # Create Navigation Process
        self.nav_queue = Queue()
        self.navigator = Navigator(self.config, self.log_queue, self.nav_queue)
        self.navigator.start()
        
        #* ====================================================================================== *#
        # Create Individual Channel Processes
        # TODO: this is hard-coded
        prn = [1,7,14,17,19,21,30] # [1,3,7,8,13,14,17,19,21,30]
        self.SpawnChannels(self.config['CHANNELS']['signals'], self.config['CHANNELS']['max_channels'])
        for i in range(len(prn)):
            self.channels[i].SetSatellite(prn[i])
            self.channels[i].start()
        #* ====================================================================================== *#
        
        # start runtime timer
        self.log_update_ms = 1000
        self.nav_update_ms = 1000 / self.config['MEASUREMENTS']['frequency']
        self.start_t = time.time()
        return
    
    def __del__(self):
        # record total execution time
        s = self.GetTimeString(1000 * (time.time() - self.start_t))
        self.log.info(f"Total Time = {s} s")
        
        # kill remaining processes
        self.log_queue.put(None)
        self.nav_queue.put(None)
        # self.logger.join()
        # self.navigator.join()
        return
    
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
                            nav_queue=self.nav_queue,
                            start_barrier=self.start_barrier,
                            done_barrier=self.done_barrier,
                            num=i
                        )
                    )
                
                # log channel spawning
                self.log.debug(f"Channel Controller: {channel_id} spawned.")
                channel_count += 1
        return
    
    def start(self):
        for ms_processed in range(0, 
                                  self.config['GENERAL']['ms_to_process']+1, 
                                  self.config['GENERAL']['ms_read_size']):
            # read next chunk of data
            self.rfbuffer.NextChunk()
            
            # inform channel barrier, process data
            self.start_barrier.wait()
            
            # check for navigation update
            if not (ms_processed % self.nav_update_ms):
                self.nav_queue.put(True)
            
            # check for screen updates
            if not (ms_processed % self.log_update_ms):
                s0 = self.GetTimeString(1000 * (time.time() - self.start_t))
                s1 = self.GetTimeString(ms_processed)
                self.log.info(f"File Time - {s1} ... System Time - {s0}")
            
            # wait for all channels to execute
            self.done_barrier.wait()
        return
    
    def run(self):
        return
    
    def GetTimeString(self, dt):
        """
        Creates a string of the form hh:mm:ss.ms
        """
        sec, ms = divmod(int(dt), 1000)
        min, sec = divmod(sec, 60)
        hr, min = divmod(min, 60)
        s = f"{hr:02d}:{min:02d}:{sec:02d}.{ms:03d}"
        return s