"""**logger.py**

======  ============================================================================================
file    sturdr/logger.py
brief   Basic logging utility for the receiver.
date    October 2024
======  ============================================================================================
"""

import logging
import csv
from multiprocessing import Process, Queue
from dataclasses import asdict
from pathlib import Path

from sturdr.channel.channel import ChannelPacket
from sturdr.nav.ephemeris import Ephemerides
from sturdr.rcvr.navigator import NavResult
from sturdr.utils.iotools import EnsurePathExists

# from pprint import pprint
# import yaml, time

# TODO: add support for logging channel data to csv files       - DONE
# TODO: add support for logging ephemeris to csv files          - DONE
# TODO: add support for logging navigation data to csv file

# ================================================================================================ #

class ColorFormatter(logging.Formatter):
    """
    A formatter to add colors to log levels.
    """
    __slots__ = 'LEVELS'
    LEVELS   : dict
    
    def __init__(self):
        logging.Formatter.__init__(self, fmt="[%(asctime)s.%(msecs)03d] [%(levelname)s] %(message)s", datefmt='%Y-%m-%d %H:%M:%S')
        
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

        # "[%(asctime)s][%(levelname)s] - %(message)s"
        # formatter = logging.Formatter("[{asctime}][{levelname}] {message}", style="{")
        self.LEVELS = \
        {
            logging.DEBUG    : f"{C}debug{RESET}",
            logging.INFO     : f"{G}info{RESET}",
            logging.WARNING  : f"{Y}warning{RESET}",
            logging.ERROR    : f"{R}error{RESET}",
            logging.CRITICAL : f"{BOLD}{M}critical{RESET}",
        }
        
    def format(self, record):
        record.levelname = self.LEVELS[record.levelno]
        return logging.Formatter.format(self, record)
    
# ================================================================================================ #

def Logger(config: dict, queue: Queue):
    """
    A thread safe logging module. Execute as:
        log_process = Process(target=Logger, args=(config,queue))
        log_process.start()
    """

    # find/create logger
    root = logging.getLogger(name='SturDR_Logger')
    root.setLevel(logging.DEBUG)

    # Use custom color console/terminal logger
    console = logging.StreamHandler()
    console.setFormatter(ColorFormatter())
    root.addHandler(console)

    # create output filenames for file logging
    path = Path(config['GENERAL']['out_folder'])
    EnsurePathExists(path)
    status_fn = path / f"{config['GENERAL']['scenario']}_ChannelStatus.csv"
    ephem_fn  = path / f"{config['GENERAL']['scenario']}_Ephemeris.csv"
    nav_fn    = path / f"{config['GENERAL']['scenario']}_Navigation.csv"

    # open the log files
    with (open(status_fn, 'w', newline='') as status_file, \
          open(ephem_fn, 'w', newline='') as ephem_file, \
          open(nav_fn, 'w', newline='') as nav_file):
        
        # initialize the writers 
        status_writer = csv.DictWriter(status_file, fieldnames=asdict(ChannelPacket()).keys())
        ephem_writer = csv.DictWriter(ephem_file, fieldnames=asdict(Ephemerides()).keys())
        nav_writer = csv.DictWriter(nav_file, fieldnames=asdict(NavResult()).keys())
        status_writer.writeheader()
        ephem_writer.writeheader()
        nav_writer.writeheader()

        # run
        while True:
            try:
                # block until message is consumed
                record = queue.get()

                # this packet should be saved to the channel status CSV file
                if isinstance(record, ChannelPacket):
                    status_writer.writerow(asdict(record))
                    status_file.flush()
                
                # this packet should be saved to the ephemeris CSV file
                elif isinstance(record, Ephemerides):
                    ephem_writer.writerow(asdict(record))
                    ephem_file.flush()
                    
                # this packet should be saved to the navigation CSV file
                elif isinstance(record, NavResult):
                    nav_writer.writerow(asdict(record))
                    nav_file.flush()

                # check for termination
                elif record is None:
                    break

                # log specified message
                else:
                    root.handle(record)

            except:
                import sys, traceback
                print('Error in logger process', file=sys.stderr)
                traceback.print_exc(file=sys.stderr)
    return

# class Logger(Process):
#     """
#     A thread safe logging module
#     """
#     __slots__ = 'queue', 'root', 'status_fn', 'ephem_fn', 'nav_fn'
#     queue     : Queue
#     root      : logging.Logger
#     status_fn : Path
#     ephem_fn  : Path
#     nav_fn    : Path

#     def __init__(self, config: dict, queue: Queue):
#         Process.__init__(self, name='SturDR_Logger_Process', daemon=True)
#         self.queue = queue

#         # find/create logger
#         self.root = logging.getLogger(name='SturDR_Logger')
#         self.root.setLevel(logging.DEBUG)

#         # Use custom color console/terminal logger
#         console = logging.StreamHandler()
#         console.setFormatter(ColorFormatter())
#         self.root.addHandler(console)

#         # create output filenames for file logging
#         path = Path(config['GENERAL']['out_folder'])
#         self.status_fn = path / f"{config['GENERAL']['scenario']}_ChannelStatus.csv"
#         self.ephem_fn  = path / f"{config['GENERAL']['scenario']}_Ephemeris.csv"
#         self.nav_fn    = path / f"{config['GENERAL']['scenario']}_Navigation.csv"

#         return
    
#     def run(self):
#         # open the log files
#         with (open(self.status_fn, 'w', newline='') as status_file, \
#               open(self.ephem_fn, 'w', newline='') as ephem_file, \
#               open(self.nav_fn, 'w', newline='') as nav_file):
            
#             # initialize the writers 
#             status_writer = csv.DictWriter(status_file, fieldnames=asdict(ChannelPacket()).keys())
#             ephem_writer = csv.DictWriter(ephem_file, fieldnames=asdict(Ephemerides()).keys())
#             nav_writer = csv.DictWriter(nav_file, fieldnames=asdict(NavResult()).keys())
#             status_writer.writeheader()
#             ephem_writer.writeheader()
#             nav_writer.writeheader()

#             # run
#             while True:
#                 try:
#                     record = self.queue.get()

#                     # this packet should be saved to the channel status CSV file
#                     if isinstance(record, ChannelPacket):
#                         status_writer.writerow(asdict(record))
#                         status_file.flush()
                    
#                     # this packet should be saved to the ephemeris CSV file
#                     elif isinstance(record, Ephemerides):
#                         ephem_writer.writerow(asdict(record))
#                         ephem_file.flush()
                        
#                     # this packet should be saved to the navigation CSV file
#                     elif isinstance(record, NavResult):
#                         nav_writer.writerow(asdict(record))
#                         nav_file.flush()

#                     # check for termination
#                     elif record is None:
#                         break

#                     # log specified message
#                     else:
#                         self.root.handle(record)

#                 except:
#                     import sys, traceback
#                     print('Error in logger process', file=sys.stderr)
#                     traceback.print_exc(file=sys.stderr)
#         return
