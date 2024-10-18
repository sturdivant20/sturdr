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
import logging.handlers
import csv
from multiprocessing import Process, Queue
from dataclasses import asdict
from sturdr.channel.channel import ChannelPacket

# TODO: add support for logging channel data to csv files
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
    __slots__ = 'queue', 'log_filename', 'log_file', 'writer'
    queue : Queue
    log_filename : str
    log_file : io.BufferedWriter
    writer : csv.DictWriter
    
    def __init__(self, config: dict, queue: Queue, name: str='SturDR_Logger', level: logging._levelToName=logging.DEBUG):
        logging.Logger.__init__(self, name, level)
        Process.__init__(self, name=name)
        
        # create a log file
        EnsurePathExists(config['GENERAL']['out_folder'])
        self.log_filename = f"{config['GENERAL']['out_folder']}/{config['GENERAL']['scenario']}.csv"
        self.log_file = open(self.log_filename, 'w', newline='')
        fields = ['ChannelNum', 'Constellation', 'Signal', 'ID', 'State', 'CodeLock', \
                  'CarrierLock', 'DataLock', 'Ephemeris', 'Week', 'ToW', 'CNo', 'Doppler', \
                  'SampleCount', 'IP', 'QP', 'IE', 'QE', 'IL', 'QL', 'IN', 'QN']
        self.writer = csv.DictWriter(self.log_file, fieldnames=fields)
        self.writer.writeheader()
        
        # attach to the logging queue
        self.queue = queue
        
        # create the iostream handler for terminal printing
        console = logging.StreamHandler()
        console.setFormatter(ColorFormatter())
        self.addHandler(console)
        return
    
    def __del__(self):
        self.log_file.close()
        return
    
    def run(self):
        while True:
            msg = self.queue.get()
            if isinstance(msg, ChannelPacket):
                # this packet should be saved to the CSV file
                self.writer.writerow(asdict(msg))
                
            elif msg is None:
                # this is the message to terminate the logger
                break
            
            else:
                # else it is a standard log message to be print to the terminal
                self.handle(msg)
        return
    
# if __name__ == '__main__':
#     # logger queue
#     queue = Queue()
    
#     # open this process's copy of the logger
#     logger = logging.getLogger('SturDR_Logger')
#     logger.addHandler(logging.handlers.QueueHandler(queue))
#     logger.setLevel(logging.DEBUG)
    
#     # handle the logging in another thread
#     p = Logger(queue)
#     p.start()
    
#     # send logging messages
#     logger.debug("debug message")
#     logger.info("info message")
#     logger.warning("warning message")
#     logger.error("error message")
#     logger.critical("critical message")
    
#     # send the final message
#     queue.put(None)
#     p.join()