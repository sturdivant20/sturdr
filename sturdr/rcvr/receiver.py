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
import multiprocessing.connection
from multiprocessing import Queue, Barrier, Process, Event, Pipe
from pathlib import Path
import numpy as np

from sturdr.rcvr.navigator import Navigator
from sturdr.rcvr.logger import Logger
from sturdr.rcvr.rf_data_buffer import RfDataBuffer
from sturdr.channel import channel, gps_l1ca_channel
from sturdr.utils.enums import GnssSignalTypes

class Reciever:
    
    __slots__ = 'config', 'rfbuffer', 'start_t', 'log_update_ms', 'nav_update_ms', 'log_process', 'logger', \
                'log_queue', 'start_barrier', 'done_barrier', 'channels', 'num_channels', 'navigator', \
                'nav_queue', 'vt_events', 'vt_pipes'
    config        : dict
    rfbuffer      : RfDataBuffer
    start_t       : float
    log_update_ms : int
    nav_update_ms : int
    logger        : logging.Logger
    log_process   : Process #Logger
    log_queue     : Queue
    start_barrier : multiprocessing.synchronize.Barrier
    done_barrier  : multiprocessing.synchronize.Barrier
    channels      : list[channel.Channel]
    num_channels  : int
    navigator     : Navigator
    nav_queue     : Queue
    vt_events     : list[multiprocessing.synchronize.Event]
    vt_pipes      : list[multiprocessing.connection.Connection]
    
    def __init__(self, config_file: Path | str):
        # Load Configuration
        with open(config_file, 'r') as file:
            self.config = yaml.safe_load(file)
        tmp = self.config['GENERAL']['log_level'].casefold()
        if tmp == 'debug':
            self.config['GENERAL']['log_level'] = logging.DEBUG
        elif tmp == 'info':
            self.config['GENERAL']['log_level'] = logging.INFO
        elif tmp == 'warn' or tmp == 'warning':
            self.config['GENERAL']['log_level'] = logging.WARNING
        elif tmp == 'error':
            self.config['GENERAL']['log_level'] = logging.ERROR
            
        # open rf data buffer
        self.rfbuffer = RfDataBuffer(self.config)
        
        # Initialize channel parameters
        self.channels = []
        self.num_channels = sum(self.config['CHANNELS']['max_channels'])
        self.start_barrier = Barrier(self.num_channels + 1)
        self.done_barrier = Barrier(self.num_channels + 1)

        # Create separate Logging process and copy for this process
        self.log_queue = Queue()
        self.logger = logging.getLogger('SturDR_Logger')
        self.logger.addHandler(logging.handlers.QueueHandler(self.log_queue))
        self.logger.setLevel(self.config['GENERAL']['log_level'])
        self.log_process = Process(target=Logger, args=(self.config, self.log_queue))
        # self.log_process = Logger(self.config, self.log_queue)
        self.log_process.start()
        
        # Create Navigation Process
        self.nav_queue = Queue()
        self.navigator = Navigator(self.config, self.log_queue, self.nav_queue)
        self.navigator.start()
        self.logger.debug(f"Navigation process started.")
        
        #* ====================================================================================== *#
        # Create Individual Channel Processes
        # TODO: this is hard-coded
        prn = [1,7,14,17,19,21,30] # [1,3,7,8,13,14,17,19,21,30]
        self.SpawnChannels(self.config['CHANNELS']['signals'], self.config['CHANNELS']['max_channels'])
        for i in range(len(prn)):
            self.channels[i].SetSatellite(prn[i])
            self.channels[i].start()
            self.logger.debug(f"Channel {i} process started.")
        #* ====================================================================================== *#

        # start runtime timer
        self.log_update_ms = 1000
        self.nav_update_ms = 1000 / self.config['MEASUREMENTS']['frequency']
        self.start_t = time.time()
        return
    
    def __del__(self):
        # record total execution time
        s = self.GetTimeString(1000 * (time.time() - self.start_t))
        self.logger.info(f"Total Execution Time = {s} s")

        # kill remaining processes
        self.log_queue.put(None)
        self.nav_queue.put(None)
        self.log_process.join()
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
                            log_queue=self.log_queue,
                            nav_queue=self.nav_queue,
                            start_barrier=self.start_barrier,
                            done_barrier=self.done_barrier,
                            num=i
                        )
                    )
                
                # log channel spawning
                channel_count += 1
        return
    
    def start(self):
        for ms_processed in range(0, 
                                  self.config['GENERAL']['ms_to_process']+1, 
                                  self.config['GENERAL']['ms_read_size']):
            # read next chunk of data
            self.rfbuffer.NextChunk()
            # self.logger.warning(f"receiver: {self.rfbuffer.buffer}")
            
            # inform channel barrier, process data
            self.start_barrier.wait()
            
            # check for navigation update
            if not (ms_processed % self.nav_update_ms):
                self.nav_queue.put(True)
            
            # check for screen updates
            if not (ms_processed % self.log_update_ms):
                s0 = self.GetTimeString(1000 * (time.time() - self.start_t))
                s1 = self.GetTimeString(ms_processed)
                self.logger.info(f"File Time - {s1} ... System Time - {s0}")
            
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