"""**logger.py**

======  ============================================================================================
file    sturdr/logger.py
brief   Basic logging utility for the receiver.
date    October 2024
======  ============================================================================================
"""

import os
import io
import logging
import csv
from multiprocessing import Process, Queue
from dataclasses import asdict
from pathlib import Path

from sturdr.channel.channel import ChannelPacket
from sturdr.nav.ephemeris import Ephemerides

# from pprint import pprint
# import yaml, time

# TODO: add support for logging channel data to csv files       - DONE
# TODO: add support for logging ephemeris to csv files          - DONE
# TODO: add support for logging navigation data to csv file

# ansi escape colors
W = "\u001b[37m"    # white
B = "\u001b[34m"    # blue
M = "\u001b[35m"    # magenta
C = "\u001b[36m"    # cyan
G = "\u001b[32m"    # green
Y = "\u001b[33m"    # yellow
R = "\u001b[31m"    # red
BOLD = "\u001b[1m"
RESET = "\u001b[0m"

def EnsurePathExists(path: str):
    """
    Make sure directory chosen exists

    Parameters
    ----------
    path : str
        path to check
    """
    os.makedirs(os.path.realpath(path), exist_ok=True)

# ================================================================================================ #

class ColorFormatter(logging.Formatter):
    """
    A formatter to add colors to log levels.
    """
    __slots__ = 'LEVELS'
    LEVELS   : dict
    
    def __init__(self):
        logging.Formatter.__init__(self, fmt="[%(asctime)s.%(msecs)03d] [%(levelname)s] %(message)s", datefmt='%Y-%m-%d %H:%M:%S')
        
        # "[%(asctime)s][%(levelname)s] - %(message)s"
        # formatter = logging.Formatter("[{asctime}][{levelname}] {message}", style="{")
        self.LEVELS = \
        {
            logging.DEBUG    : f"{C}debug{W}",
            logging.INFO     : f"{G}info{W}",
            logging.WARNING  : f"{Y}warning{W}",
            logging.ERROR    : f"{R}error{W}",
            logging.CRITICAL : f"{M}critical{W}",
        }
        
    def format(self, record):
        record.levelname = self.LEVELS[record.levelno]
        return logging.Formatter.format(self, record)
    
# ================================================================================================ #

class Logger(logging.Logger, Process):
    """
    A thread safe logging module.
    """
    __slots__ = 'queue', 'status_filename', 'ephemeris_filename', 'status_file', 'ephemeris_file', \
                'status_writer', 'ephemeris_writer'
    queue              : Queue
    status_filename    : Path
    ephemeris_filename : Path
    status_file        : io.BufferedWriter
    ephemeris_file     : io.BufferedWriter
    status_writer      : csv.DictWriter
    ephemeris_writer   : csv.DictWriter
    
    def __init__(self, 
                 config: dict, 
                 queue: Queue, 
                 name: str='SturDR_Logger', 
                 level: logging._levelToName=logging.DEBUG):
        logging.Logger.__init__(self, name, level)
        Process.__init__(self, name=name)
        EnsurePathExists(config['GENERAL']['out_folder'])
        
        # attach to the logging queue
        self.queue = queue
        
        # create log file names
        path = Path(config['GENERAL']['out_folder'])
        self.status_filename = path / f"{config['GENERAL']['scenario']}_ChannelStatus.csv"
        self.ephemeris_filename = path / f"{config['GENERAL']['scenario']}_Ephemeris.csv"
        
        # create the iostream handler for terminal printing
        console = logging.StreamHandler()
        console.setFormatter(ColorFormatter())
        self.addHandler(console)
        return
    
    def run(self):
        # open the CSV log files
        with open(self.status_filename, 'w', newline='') as status_file, \
             open(self.ephemeris_filename, 'w', newline='') as ephemeris_file:
            
            # initialize the writers 
            status_writer = csv.DictWriter(status_file, fieldnames=asdict(ChannelPacket()).keys())
            ephemeris_writer = csv.DictWriter(ephemeris_file, fieldnames=asdict(Ephemerides()).keys())
            status_writer.writeheader()
            ephemeris_writer.writeheader()
            
            while True:
                msg = self.queue.get()

                if isinstance(msg, ChannelPacket):
                    # this packet should be saved to the channel status CSV file
                    status_writer.writerow(asdict(msg))
                
                elif isinstance(msg, Ephemerides):
                    # this packet should be saved to the ephemeris CSV file
                    ephemeris_writer.writerow(asdict(msg))
                    
                elif msg is None:
                    # this is the message to terminate the logger
                    break
                
                else:
                    # else it is a standard log message to be print to the terminal
                    self.handle(msg)
        return
    
# if __name__ == '__main__':
#     # Load Configuration
#     config_file = './config/gps_l1ca_rcvr.yaml'
#     with open(config_file, 'r') as file:
#         config = yaml.safe_load(file)
        
#     # logger queue
#     queue = Queue()
    
#     # handle the logging in another thread
#     p = Logger(config, queue)
#     p.start()
    
#     # open this process's copy of the logger
#     logger = logging.getLogger('SturDR_Logger')
#     logger.addHandler(logging.handlers.QueueHandler(queue))
#     logger.setLevel(logging.DEBUG)
    
#     # send logging messages
#     time.sleep(1)
#     logger.debug("debug message")
#     logger.info("info message")
#     logger.warning("warning message")
#     logger.error("error message")
#     logger.critical("critical message")
#     queue.put(Ephemerides())
#     queue.put(ChannelPacket())
    
#     # send the final message
#     queue.put(None)
#     p.join()